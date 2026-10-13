// Nodo ROS 2: control por corriente de los Talon SRX de los joints 1 y 4 desde la Jetson (Phoenix 5).
//
// Subscribers (uno por joint, <name> = joint1, joint4):
//   /arm_teleop/<name>/current  (std_msgs/Float64)  setpoint de corriente en AMPERES
//
// Publishers (uno por joint):
//   /arm_feedback/<name>_deg    (std_msgs/Float64)  posicion actual en grados (0 a 360) con offset y soporte para negativos
//   /arm_feedback/<name>_ticks  (std_msgs/Float64)  posicion actual en ticks (0 a 4095) en teoria
//
// Parametros del nodo (declare_parameter):
//   joint_names   string[] (default ["joint1","joint4"])
//   loop_hz       double   (default 200.0)
//
// Parametros por joint (prefijo <name>., por ejemplo joint1.can_id):
//   can_id        int    (default 7 para el 1er joint de joint_names, 8 para el 2do)
//   slot          int    (default 0)
//   kP            double (default 1.0)   escribe el kP en el Talon al arrancar
//   watchdog_ms   int    (default 500)   si no llega current_cmd en este tiempo, baja a 0 A
//   max_amps      double (default 5.0)   satura el setpoint
//   phys_offset   double (default 0.0)   offset en grados
//   sign          double (default -1.0)  signo de la conversion ticks->grados (regla de la mano derecha)
//   required      bool   (default false) si es true y el Talon no responde al arrancar, el nodo no arranca;
//                                        si es false, el joint queda inactivo y los demas siguen funcionando
//
// Politica de falla (isolate): si un Talon reporta falla (UV, HW, RstEn, APIErr) ese joint se aisla:
// pasa a salida neutra (PercentOutput 0) y ya no obedece comandos; los demas joints siguen normales.
// El aislamiento queda latcheado: para recuperar el joint hay que reiniciar el nodo.
//
// Encoder: Mag Encoder absoluto, 12 bits, 4096 ticks por vuelta, en la flecha final.
// La publicacion de ticks y deg es ACTUAL con un offset y soporte para grados y ticks negativos.

#define Phoenix_No_WPI

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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

// Con sign = -1.0 (default) es identica a la version de un solo joint: offset - ticks*360/4096
static double ticks_to_deg(double t, double offset, double sign) {
    return offset + sign * (t * 360.0 / COUNTS_PER_REV);
}

// ============================= JOINT DRIVER ===========================
// Todo lo que es por joint: Talon, parametros, setpoint, watchdog, sub y pubs.

class JointDriver {
public:
    JointDriver(rclcpp::Node& node, const std::string& name, int default_can_id)
        : logger_(node.get_logger()), name_(name) {
        // Parametros
        const std::string p = name + ".";
        can_id_      = node.declare_parameter<int>(p + "can_id", default_can_id);
        slot_        = node.declare_parameter<int>(p + "slot", 0);
        kp_          = node.declare_parameter<double>(p + "kP", 1.0);
        watchdog_ms_ = node.declare_parameter<int>(p + "watchdog_ms", 500);
        max_amps_    = node.declare_parameter<double>(p + "max_amps", 5.0);
        phys_offset_ = node.declare_parameter<double>(p + "phys_offset", 0.0);
        sign_        = node.declare_parameter<double>(p + "sign", -1.0);
        required_    = node.declare_parameter<bool>(p + "required", false);

        RCLCPP_INFO(logger_,
            "%s: can_id=%d slot=%d kP=%.3f watchdog=%dms max=%.2fA offset=%.3f sign=%.1f required=%d",
            name_.c_str(), can_id_, slot_, kp_, watchdog_ms_, max_amps_, phys_offset_, sign_,
            static_cast<int>(required_));

        // Talon
        talon_ = std::make_unique<TalonSRX>(can_id_);

        // Reintento con tiempo limite (2 s): el primer frame del Talon puede tardar en llegar
        const auto deadline = Clock::now() + std::chrono::seconds(2);
        bool talon_ok = false;
        for (;;) {
            if (talon_->GetFirmwareVersion() != -1) { talon_ok = true; break; }
            if (Clock::now() >= deadline) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!talon_ok) {
            if (required_) {
                RCLCPP_FATAL(logger_, "%s: Talon ID %d no responde en can0", name_.c_str(), can_id_);
                throw std::runtime_error(name_ + ": Talon no responde");
            }
            RCLCPP_WARN(logger_,
                "%s: Talon ID %d no responde en can0, joint INACTIVO (required=false)",
                name_.c_str(), can_id_);
            return;   // active_ se queda en false, sin sub ni pubs
        }

        // Setup del sensor y gains
        talon_->ClearStickyFaults(50);
        talon_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
        talon_->SetSensorPhase(false);
        talon_->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10, 10);
        talon_->SelectProfileSlot(slot_, 0);
        talon_->Config_kP(slot_, kp_, 50);

        // Marca de "nunca ha llegado un comando"
        last_cmd_time_ = Clock::now() - std::chrono::seconds(10);
        setpoint_a_    = 0.0;

        // Pub y sub
        sub_cmd_ = node.create_subscription<std_msgs::msg::Float64>(
            "/arm_teleop/" + name_ + "/current", 10,
            std::bind(&JointDriver::on_current_cmd, this, std::placeholders::_1));
        pub_deg_   = node.create_publisher<std_msgs::msg::Float64>("/arm_feedback/" + name_ + "_deg", 10);
        pub_ticks_ = node.create_publisher<std_msgs::msg::Float64>("/arm_feedback/" + name_ + "_ticks", 10);

        active_ = true;
    }

    bool active() const { return active_; }

    // Lazo de control periodico (lo llama el timer del nodo)
    void tick() {
        if (!active_) return;

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
                    RCLCPP_WARN(logger_,
                        "%s: watchdog: sin current_cmd en %d ms, bajando a 0 A",
                        name_.c_str(), watchdog_ms_);
                    watchdog_warned_ = true;
                }
            }
        }
        if (amps != 0.0) watchdog_warned_ = false;

        if (faulted_) {
            talon_->Set(ControlMode::PercentOutput, 0.0);   // joint aislado: neutro, ignora comandos
        } else {
            talon_->Set(ControlMode::Current, amps);
        }

        // Leer encoder y publicar posicion ACTUAL
        //Nota: no nos conviene que sea modulo, porque cuando llega a 0 o pasa de 360
        //      la cuenta pierde referencia de ticks y grados, y eso hace que el control se vuelva loco.
        const double ticks = static_cast<double>(talon_->GetSelectedSensorPosition());
        const double deg = ticks_to_deg(ticks, phys_offset_, sign_);

        std_msgs::msg::Float64 msg_ticks; msg_ticks.data = ticks;
        std_msgs::msg::Float64 msg_deg;   msg_deg.data   = deg;
        pub_ticks_->publish(msg_ticks);
        pub_deg_->publish(msg_deg);
    }

    // Lectura de fallas (la llama el hilo de fallas del nodo)
    void check_faults() {
        if (!active_) return;
        Faults f; talon_->GetFaults(f);
        if (f.UnderVoltage || f.HardwareFailure || f.ResetDuringEn || f.APIError) {
            RCLCPP_WARN(logger_,
                "%s: falla: UV=%d HW=%d RstEn=%d APIErr=%d",
                name_.c_str(), f.UnderVoltage, f.HardwareFailure, f.ResetDuringEn, f.APIError);
            if (!faulted_.exchange(true)) {
                RCLCPP_ERROR(logger_,
                    "%s: JOINT AISLADO por falla, salida neutra (los demas joints siguen)",
                    name_.c_str());
            }
        }
    }

    // Piezas del apagado seguro (las usa el destructor del nodo)
    void zero_current() {
        if (!active_) return;
        talon_->Set(ControlMode::Current, 0.0);
    }
    void zero_percent() {
        if (!active_) return;
        talon_->Set(ControlMode::PercentOutput, 0.0);
    }

private:
    // Callback del topico de setpoint
    void on_current_cmd(const std_msgs::msg::Float64::SharedPtr msg) {
        double v = msg->data;
        if (std::fabs(v) > max_amps_) {
            RCLCPP_WARN(logger_,
                "%s: setpoint %.3f A saturado a +/- %.3f A", name_.c_str(), v, max_amps_);
            v = (v > 0 ? max_amps_ : -max_amps_);
        }
        std::lock_guard<std::mutex> lock(cmd_mtx_);
        setpoint_a_    = v;
        last_cmd_time_ = Clock::now();
    }

    rclcpp::Logger logger_;
    std::string    name_;

    // Parametros
    int    can_id_;
    int    slot_;
    double kp_;
    int    watchdog_ms_;
    double max_amps_;
    double phys_offset_;
    double sign_;
    bool   required_;
    bool   active_ = false;
    std::atomic<bool> faulted_{false};   // latch de falla (isolate)

    // Talon
    std::unique_ptr<TalonSRX> talon_;

    // Estado del setpoint
    std::mutex cmd_mtx_;
    double     setpoint_a_ = 0.0;
    Clock::time_point last_cmd_time_;
    bool       watchdog_warned_ = false;

    // Pub y sub
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_deg_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    pub_ticks_;
};

// ============================= NODO ===================================
// Lo que es global: interfaz CAN, hilo de enable, hilo de fallas y timer.

class TalonJointsNode : public rclcpp::Node {
public:
    TalonJointsNode() : Node("talon_joints_node") {
        // Parametros
        const std::vector<std::string> default_names{"joint1", "joint4"};
        const std::vector<std::string> joint_names =
            this->declare_parameter<std::vector<std::string>>("joint_names", default_names);
        loop_hz_ = this->declare_parameter<double>("loop_hz", 200.0);

        RCLCPP_INFO(this->get_logger(),
            "talon_joints_node: %zu joints, loop=%.1fHz", joint_names.size(), loop_hz_);

        // CAN y Talons (si un joint requerido no responde, lanza excepcion antes de arrancar hilos)
        ctre::phoenix::platform::can::SetCANInterface("can0");
        for (size_t i = 0; i < joint_names.size(); ++i) {
            joints_.push_back(std::make_unique<JointDriver>(
                *this, joint_names[i], 7 + static_cast<int>(i)));
        }

        // Threads
        run_     = true;
        enabled_ = true;
        enable_thread_ = std::thread(&TalonJointsNode::enable_loop, this);
        fault_thread_  = std::thread(&TalonJointsNode::fault_loop,  this);

        // Timer del lazo de control
        const auto period = std::chrono::microseconds(static_cast<long>(1e6 / loop_hz_));
        timer_ = this->create_wall_timer(period, std::bind(&TalonJointsNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "listo");
    }

    ~TalonJointsNode() override {
        // Apagado seguro: 0 A por 200 ms antes de bajar los threads
        for (int i = 0; i < 20; ++i) {
            for (auto& j : joints_) j->zero_current();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        for (auto& j : joints_) j->zero_percent();

        enabled_ = false;
        run_     = false;
        if (enable_thread_.joinable()) enable_thread_.join();
        if (fault_thread_.joinable())  fault_thread_.join();
    }

private:
    // Lazo de control periodico: un tick por joint
    void tick() {
        for (auto& j : joints_) j->tick();
    }

    void enable_loop() {
        while (run_) {
            if (enabled_) ctre::phoenix::unmanaged::FeedEnable(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void fault_loop() {
        while (run_) {
            for (auto& j : joints_) j->check_faults();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // Parametros
    double loop_hz_;

    // Joints
    std::vector<std::unique_ptr<JointDriver>> joints_;

    // Threads y timer
    std::atomic<bool> run_{false};
    std::atomic<bool> enabled_{false};
    std::thread enable_thread_;
    std::thread fault_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        auto node = std::make_shared<TalonJointsNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "fatal: %s", e.what());
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}