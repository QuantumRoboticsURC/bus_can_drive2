#define Phoenix_No_WPI

#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using Twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;

// ==========================================================================
// Motor declarations — IDs from ROS2 parameters (defaults match working code)
// ==========================================================================

// CAN IDs (overridden by ROS2 parameters in main())
int can_id_fl       = 1;   // Front Left
int can_id_fr       = 0;   // Front Right
int can_id_bl       = 3;   // Back Left
int can_id_br       = 2;   // Back Right

// Drive — constructed after params are read
std::unique_ptr<TalonFX> talFrontLeft_ptr;
std::unique_ptr<TalonFX> talFrontRight_ptr;
std::unique_ptr<TalonFX> talBackLeft_ptr;
std::unique_ptr<TalonFX> talBackRight_ptr;

// References that keep the original `.` access syntax untouched in the rest of the code
#define talFrontLeft  (*talFrontLeft_ptr)
#define talFrontRight (*talFrontRight_ptr)
#define talBackLeft   (*talBackLeft_ptr)
#define talBackRight  (*talBackRight_ptr)

// ==========================================================================
// Utility
// ==========================================================================

bool start_config = true;

double phys_error=0;

// ==========================================================================
// Initialization — matching working code exactly
// ==========================================================================

void initAll() {
    // --- Drive motors ---
    talFrontLeft.ConfigFactoryDefault();
    talFrontRight.ConfigFactoryDefault();
    talBackLeft.ConfigFactoryDefault();
    talBackRight.ConfigFactoryDefault();

    talFrontLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    talFrontRight.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    talBackLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    talBackRight.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

    talFrontLeft.SetSensorPhase(false);
    talFrontLeft.SetInverted(false);
    talFrontRight.SetSensorPhase(true);
    talFrontRight.SetInverted(true);
    talBackLeft.SetSensorPhase(false);
    talBackLeft.SetInverted(false);
    talBackRight.SetSensorPhase(true);
    talBackRight.SetInverted(true);

    talFrontLeft.ConfigNominalOutputForward(0, 10);
    talFrontLeft.ConfigNominalOutputReverse(0, 10);
    talFrontLeft.ConfigPeakOutputForward(1, 10);
    talFrontLeft.ConfigPeakOutputReverse(-1, 10);

    talFrontRight.ConfigNominalOutputForward(0, 10);
    talFrontRight.ConfigNominalOutputReverse(0, 10);
    talFrontRight.ConfigPeakOutputForward(1, 10);
    talFrontRight.ConfigPeakOutputReverse(-1, 10);

    talBackLeft.ConfigNominalOutputForward(0, 10);
    talBackLeft.ConfigNominalOutputReverse(0, 10);
    talBackLeft.ConfigPeakOutputForward(1, 10);
    talBackLeft.ConfigPeakOutputReverse(-1, 10);

    talBackRight.ConfigNominalOutputForward(0, 10);
    talBackRight.ConfigNominalOutputReverse(0, 10);
    talBackRight.ConfigPeakOutputForward(1, 10);
    talBackRight.ConfigPeakOutputReverse(-1, 10);

    talFrontLeft.SelectProfileSlot(0, 0);
    talFrontLeft.Config_kF(0, 0.04721247923, 10);
    talFrontLeft.Config_kP(0, 0.16084905660, 10);
    talFrontLeft.Config_kI(0, 0.001, 10);
    talFrontLeft.Config_kD(0, 1.68, 10);

    talFrontRight.SelectProfileSlot(0, 0);
    talFrontRight.Config_kF(0, 0.04721247923, 10);
    talFrontRight.Config_kP(0, 0.16084905660, 10);
    talFrontRight.Config_kI(0, 0.001, 10);
    talFrontRight.Config_kD(0, 1.68, 10);

    talBackLeft.SelectProfileSlot(0, 0);
    talBackLeft.Config_kF(0, 0.04721247923, 10);
    talBackLeft.Config_kP(0, 0.16084905660, 10);
    talBackLeft.Config_kI(0, 0.001, 10);
    talBackLeft.Config_kD(0, 1.68, 10);

    talBackRight.SelectProfileSlot(0, 0);
    talBackRight.Config_kF(0, 0.04721247923, 10);
    talBackRight.Config_kP(0, 0.16084905660, 10);
    talBackRight.Config_kI(0, 0.001, 10);
    talBackRight.Config_kD(0, 1.68, 10);
}

// ==========================================================================
// ROS2 Node
// ==========================================================================

class DriveNode : public rclcpp::Node {
public:
    DriveNode() : Node("drive_ctre") {
        // ---- Parameters ----
        // CAN IDs (int)
        this->declare_parameter<int>("can_id_fl",      can_id_fl);
        this->declare_parameter<int>("can_id_fr",      can_id_fr);
        this->declare_parameter<int>("can_id_bl",      can_id_bl);
        this->declare_parameter<int>("can_id_br",      can_id_br);

        // Drive
        velocity_sub_ = this->create_subscription<Twist>(
            "/cmd_vel", 10, std::bind(&DriveNode::velocity_callback, this, _1));
            
        // Drive feedback
        pub_left_vel_ = this->create_publisher<std_msgs::msg::Float64>("left_targetVelocity", 10);
        pub_right_vel_ = this->create_publisher<std_msgs::msg::Float64>("right_targetVelocity", 10);


         RCLCPP_INFO(this->get_logger(),
             "CTRE node ready (Drive Only)");
    }

private:
    void velocity_callback(const Twist::SharedPtr msg) {
        double v = msg->linear.x;
        double omega = msg->angular.z;
        const double r = 0.15;
        const double b = 0.65;

        double wr = (v / r) + (omega * b / (2 * r));
        double wl = (v / r) - (omega * b / (2 * r));
        double conv = 2048.0 * 45.0 / (2.0 * 3.1416 * 10.0);

        ctre::phoenix::unmanaged::FeedEnable(5000);
        talFrontLeft.Set(ControlMode::Velocity, wl * conv);
        talBackLeft.Set(ControlMode::Velocity, wl * conv);
        talFrontRight.Set(ControlMode::Velocity, wr * conv);
        talBackRight.Set(ControlMode::Velocity, wr * conv);

        std_msgs::msg::Float64 left_msg, right_msg;
        left_msg.data = wl; right_msg.data = wr;
        pub_left_vel_->publish(left_msg);
        pub_right_vel_->publish(right_msg);
    }

    rclcpp::Subscription<Twist>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_vel_;


    rclcpp::TimerBase::SharedPtr feedback_timer_;
};

// ==========================================================================
// Main — same pattern as working code
// ==========================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (start_config) {
        // Read CAN-ID parameters from a temporary node before constructing motor objects
        auto param_node = std::make_shared<rclcpp::Node>("drive_ctre_params");
        can_id_fl      = param_node->declare_parameter<int>("can_id_fl",      can_id_fl);
        can_id_fr      = param_node->declare_parameter<int>("can_id_fr",      can_id_fr);
        can_id_bl      = param_node->declare_parameter<int>("can_id_bl",      can_id_bl);
        can_id_br      = param_node->declare_parameter<int>("can_id_br",      can_id_br);

        std::cout << "CAN IDs drive FL=" << can_id_fl
                  << " FR=" << can_id_fr
                  << " BL=" << can_id_bl
                  << " BR=" << can_id_br << std::endl;

        // Construct motor objects with parameterized IDs
        talFrontLeft_ptr  = std::make_unique<TalonFX>(can_id_fl);
        talFrontRight_ptr = std::make_unique<TalonFX>(can_id_fr);
        talBackLeft_ptr   = std::make_unique<TalonFX>(can_id_bl);
        talBackRight_ptr  = std::make_unique<TalonFX>(can_id_br);

        std::string interface = "can0";
        ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
        initAll();
        start_config = false;
    }

    // Verify CAN communication
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int fw = talFrontLeft.GetFirmwareVersion();
    if (fw == -1) {
        std::cerr << "⚠️  WARNING: talFrontLeft (ID " << can_id_fl << ") not responding on CAN bus!" << std::endl;
    } else {
        std::cout << "✅ talFrontLeft (ID " << can_id_fl << ") firmware: " << fw << std::endl;
    }

    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
