// Nodo ROS 2: control por corriente del Talon SRX del joint1 desde la Jetson (Phoenix 5).
//
// Subscriber:
//   /arm_teleop/joint1/current  (std_msgs/Float64)  setpoint de corriente en AMPERES
//
// Publishers:
//   /arm_feedback/joint1_deg    (std_msgs/Float64)  posicion actual en grados (0 a 360) con offset y soporte para negativos
//   /arm_feedback/joint1_ticks  (std_msgs/Float64)  posicion actual en ticks (0 a 4095) en teoria
//
// Parametros (declare_parameter):
//   can_id        int    (default 7)
//   slot          int    (default 0)
//   kP            double (default 1.0)   escribe el kP en el Talon al arrancar
//   loop_hz       double (default 200.0)
//   watchdog_ms   int    (default 500)   si no llega current_cmd en este tiempo, baja a 0 A
//   max_amps      double (default 5.0)   satura el setpoint
//
// Encoder: Mag Encoder absoluto, 12 bits, 4096 ticks por vuelta, en la flecha final.
// La publicacion de ticks y deg es ACTUAL con un offset y soporte para grados y ticks negativos.

#define Phoenix_No_WPI

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using Clock = std::chrono::steady_clock;

constexpr double COUNTS_PER_REV = 4096.0;
constexpr int    TICKS_PER_REV  = 4096;

// ============================= HELPERS DE ENCODER =====================

//modificado c/ offset, porque no seguia la regla de la mano derecha
static double ticks_to_deg_j1(double t,double offset) {
    return offset-(t * 360.0 / COUNTS_PER_REV);
}

// ============================= NODO ===================================

class Joint1CurrentNode : public rclcpp::Node {
public:
    Joint1CurrentNode() : Node("joint1_current_node") {
        // Parametros
        can_id_      = this->declare_parameter<int>("can_id", 7);
        slot_        = this->declare_parameter<int>("slot", 0);
        kp_          = this->declare_parameter<double>("kP", 1.0);
        loop_hz_     = this->declare_parameter<double>("loop_hz", 200.0);
        watchdog_ms_ = this->declare_parameter<int>("watchdog_ms", 500);
        max_amps_    = this->declare_parameter<double>("max_amps", 5.0);
        phys_offset_ = this->declare_parameter<double>("phys_offset", 0.0);
        

        RCLCPP_INFO(this->get_logger(),
            "joint1_current_node: can_id=%d slot=%d kP=%.3f loop=%.1fHz watchdog=%dms max=%.2fA",
            can_id_, slot_, kp_, loop_hz_, watchdog_ms_, max_amps_);

        // CAN y Talon
        ctre::phoenix::platform::can::SetCANInterface("can0");
        talon_j1 = std::make_unique<TalonSRX>(can_id_);
        // Reintento con tiempo limite (1 s): el primer frame del Talon puede tardar en llegar
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        bool talon_ok = false;
        for (;;) {
            if (talon_j1->GetFirmwareVersion() != -1) { talon_ok = true; break; }
            if (Clock::now() >= deadline) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!talon_ok) {
            RCLCPP_FATAL(this->get_logger(), "Talon ID %d no responde en can0", can_id_);
            throw std::runtime_error("Talon no responde");
        }

        // Setup del sensor y gains
        talon_j1->ClearStickyFaults(50);
        talon_j1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
        talon_j1->SetSensorPhase(false);
        talon_j1->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 10);
        talon_j1->SelectProfileSlot(slot_, 0);
        talon_j1->Config_kP(slot_, kp_, 50);

        // Marca de "nunca ha llegado un comando"
        last_cmd_time_ = Clock::now() - std::chrono::seconds(10);
        setpoint_a_    = 0.0;

        // Pub y sub
        sub_cmd_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_teleop/joint1/current", 10,
            std::bind(&Joint1CurrentNode::on_current_cmd, this, std::placeholders::_1));
        pub_deg_   = this->create_publisher<std_msgs::msg::Float64>("/arm_feedback/joint1_deg", 10);
        pub_ticks_ = this->create_publisher<std_msgs::msg::Float64>("/arm_feedback/joint1_ticks", 10);

        // Threads
        run_     = true;
        enabled_ = true;
        enable_thread_ = std::thread(&Joint1CurrentNode::enable_loop, this);
        fault_thread_  = std::thread(&Joint1CurrentNode::fault_loop,  this);

        // Timer del lazo de control
        const auto period = std::chrono::microseconds(static_cast<long>(1e6 / loop_hz_));
        timer_ = this->create_wall_timer(period, std::bind(&Joint1CurrentNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "listo");
    }

    ~Joint1CurrentNode() override {
        // Apagado seguro: 0 A por 200 ms antes de bajar los threads
        for (int i = 0; i < 20; ++i) {
            talon_j1->Set(ControlMode::Current, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        talon_j1->Set(ControlMode::PercentOutput, 0.0);

        enabled_ = false;
        run_     = false;
        if (enable_thread_.joinable()) enable_thread_.join();
        if (fault_thread_.joinable())  fault_thread_.join();
    }

private:
    // Callback del tópico de setpoint
    void on_current_cmd(const std_msgs::msg::Float64::SharedPtr msg) {
        double v = msg->data;
        if (std::fabs(v) > max_amps_) {
            RCLCPP_WARN(this->get_logger(),
                "setpoint %.3f A saturado a +/- %.3f A", v, max_amps_);
            v = (v > 0 ? max_amps_ : -max_amps_);
        }
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        setpoint_a_    = v;
        last_cmd_time_ = Clock::now();
    }

    // Lazo de control periódico
    void tick() {
        // Decidir setpoint efectivo con watchdog
        double amps = 0.0;
        {
            std::lock_guard<std::mutex> lock(cmd_mtx_);
            const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                                Clock::now() - last_cmd_time_).count();
            if (age <= watchdog_ms_) {
                amps = setpoint_a_;
            } else {
                amps = 0.0;
                if (!watchdog_warned_) {
                    RCLCPP_WARN(this->get_logger(),
                        "watchdog: sin current_cmd en %d ms, bajando a 0 A", watchdog_ms_);
                    watchdog_warned_ = true;
                }
            }
        }
        if (amps != 0.0) watchdog_warned_ = false;

        talon_j1->Set(ControlMode::Current, amps);

        // Leer encoder y publicar posicion ACTUAL
        //Nota: no nos conviene que sea modulo, porque cuando llega a 0 o pasa de 360 
        //      la cuenta pierde referencia de ticks y grados, y eso hace que el control se vuelva loco.
        const double ticks = static_cast<double>(talon_j1->GetSelectedSensorPosition());
        const double deg = ticks_to_deg_j1(ticks,phys_offset_);

        std_msgs::msg::Float64 msg_ticks; msg_ticks.data = ticks;
        std_msgs::msg::Float64 msg_deg;   msg_deg.data   = deg;
        pub_ticks_->publish(msg_ticks);
        pub_deg_->publish(msg_deg);
    }

    void enable_loop() {
        while (run_) {
            if (enabled_) ctre::phoenix::unmanaged::FeedEnable(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void fault_loop() {
        while (run_) {
            Faults f; talon_j1->GetFaults(f);
            if (f.UnderVoltage || f.HardwareFailure || f.ResetDuringEn || f.APIError) {
                RCLCPP_WARN(this->get_logger(),
                    "falla: UV=%d HW=%d RstEn=%d APIErr=%d",
                    f.UnderVoltage, f.HardwareFailure, f.ResetDuringEn, f.APIError);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // Parametros
    int    can_id_;
    int    slot_;
    double kp_;
    double loop_hz_;
    int    watchdog_ms_;
    double max_amps_;
    double phys_offset_;

    // Talon
    std::unique_ptr<TalonSRX> talon_j1;

    // Estado del setpoint
    std::mutex cmd_mtx_;
    double     setpoint_a_;
    Clock::time_point last_cmd_time_;
    bool       watchdog_warned_ = false;

    // Threads y timer
    std::atomic<bool> run_{false};
    std::atomic<bool> enabled_{false};
    std::thread enable_thread_;
    std::thread fault_thread_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Pub y sub
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_deg_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_ticks_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<Joint1CurrentNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "fatal: %s", e.what());
        ret=1;
    }
    rclcpp::shutdown();
    return ret;
}
