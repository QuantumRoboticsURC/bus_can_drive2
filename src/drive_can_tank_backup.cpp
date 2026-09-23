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
int can_id_arm1     = 7;   // Joint 1
int can_id_arm5     = 15;  // Joint 4
int can_id_gripper  = 14;  // Gripper
int can_id_lineal   = 11;  // Linear Actuator
int can_id_fl       = 2;   // Front Left
int can_id_fr       = 1;   // Front Right
int can_id_bl       = 3;   // Back Left
int can_id_br       = 0;   // Back Right

// Arm — constructed after params are read
std::unique_ptr<TalonSRX> srxArm1_ptr;
std::unique_ptr<TalonSRX> srxArm5_ptr;
std::unique_ptr<TalonSRX> srxGripper_ptr;
std::unique_ptr<TalonSRX> srxLineal_ptr;

// Drive — constructed after params are read
std::unique_ptr<TalonFX> talFrontLeft_ptr;
std::unique_ptr<TalonFX> talFrontRight_ptr;
std::unique_ptr<TalonFX> talBackLeft_ptr;
std::unique_ptr<TalonFX> talBackRight_ptr;

// References that keep the original `.` access syntax untouched in the rest of the code
#define srxArm1       (*srxArm1_ptr)
#define srxArm5       (*srxArm5_ptr)
#define srxGripper    (*srxGripper_ptr)
#define srxLineal     (*srxLineal_ptr)
#define talFrontLeft  (*talFrontLeft_ptr)
#define talFrontRight (*talFrontRight_ptr)
#define talBackLeft   (*talBackLeft_ptr)
#define talBackRight  (*talBackRight_ptr)

// Joint limits (counts and degrees) — overridden by ROS2 parameters in main()
// Joint 1
float j1_deg_min    = -85.0f;
float j1_deg_max    =  90.0f;
float j1_ticks_min  = 1847.0f;
float j1_ticks_max  =  3895.0f;
// Joint 4
float j4_deg_min    = -150.0f;
float j4_deg_max    =  150.0f;
float j4_ticks_min  =  458.0f;
float j4_ticks_max  =  3871.0f;

// ==========================================================================
// Utility
// ==========================================================================

bool start_config = true;

double phys_error=0;

double my_map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double ticks_to_deg_j1(double ticks) {
    // ticks → degrees (limits from ROS2 params)
    return my_map(ticks, j1_ticks_min, j1_ticks_max, j1_deg_min, j1_deg_max);
}

double ticks_to_deg_j4(double ticks) {
    // ticks → degrees (limits from ROS2 params, inverted)
    return -my_map(ticks, j4_ticks_min, j4_ticks_max, j4_deg_min, j4_deg_max);
}

// ==========================================================================
// Initialization — matching working code exactly
// ==========================================================================

void initAll() {
    // --- Joint 1 (srxArm1 ID 7) — from working code ---
    srxArm1.ConfigFactoryDefault();
    srxArm1.SetInverted(false);
    srxArm1.SetSensorPhase(false);  // TRUE in working code
    srxArm1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

    srxArm1.ConfigNominalOutputForward(0, 10);
    srxArm1.ConfigNominalOutputReverse(0, 10);
    srxArm1.ConfigPeakOutputForward(1, 10);
    srxArm1.ConfigPeakOutputReverse(-1, 10);

    srxArm1.Config_kF(0, 0, 10);
    srxArm1.Config_kP(0, 10, 10);
    srxArm1.Config_kI(0, 0.0000, 10);
    srxArm1.Config_kD(0, 0.0, 10);
    srxArm1.ConfigMotionCruiseVelocity(25, 10);
    srxArm1.ConfigMotionAcceleration(35, 10);

    // --- Joint 4 (srxArm5 ID 15) — from working code ---
    srxArm5.ConfigFactoryDefault();
    srxArm5.SetInverted(false);
    srxArm5.SetSensorPhase(true);
    srxArm5.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

    srxArm5.ConfigNominalOutputForward(0, 10);
    srxArm5.ConfigNominalOutputReverse(0, 10);
    srxArm5.ConfigPeakOutputForward(1, 10);
    srxArm5.ConfigPeakOutputReverse(-1, 10);

    srxArm5.Config_kF(0, 0.0, 10);
    srxArm5.Config_kP(0, 30.0, 10);
    srxArm5.Config_kI(0, 0.001, 10);
    srxArm5.Config_kD(0, 50.0, 10);
    srxArm5.ConfigMotionCruiseVelocity(30, 10);
    srxArm5.ConfigMotionAcceleration(40, 10);

    // --- Gripper ---
    srxGripper.ConfigFactoryDefault();
    srxGripper.ConfigNominalOutputForward(0, 10);
    srxGripper.ConfigNominalOutputReverse(0, 10);
    srxGripper.ConfigPeakOutputForward(1, 10);
    srxGripper.ConfigPeakOutputReverse(-1, 10);

    // --- Linear Actuator ---
    srxLineal.ConfigFactoryDefault();
    srxLineal.ConfigNominalOutputForward(0, 10);
    srxLineal.ConfigNominalOutputReverse(0, 10);
    srxLineal.ConfigPeakOutputForward(1, 10);
    srxLineal.ConfigPeakOutputReverse(-1, 10);

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

class ArmDriveNode : public rclcpp::Node {
public:
    ArmDriveNode() : Node("arm_drive_ctre") {
        // ---- Parameters ----
        // CAN IDs (int)
        this->declare_parameter<int>("can_id_arm1",    can_id_arm1);
        this->declare_parameter<int>("can_id_arm5",    can_id_arm5);
        this->declare_parameter<int>("can_id_gripper", can_id_gripper);
        this->declare_parameter<int>("can_id_lineal",  can_id_lineal);
        this->declare_parameter<int>("can_id_fl",      can_id_fl);
        this->declare_parameter<int>("can_id_fr",      can_id_fr);
        this->declare_parameter<int>("can_id_bl",      can_id_bl);
        this->declare_parameter<int>("can_id_br",      can_id_br);

        // Joint 1 limits (float / double)
        this->declare_parameter<double>("j1_deg_min",   j1_deg_min);
        this->declare_parameter<double>("j1_deg_max",   j1_deg_max);
        this->declare_parameter<double>("j1_ticks_min", j1_ticks_min);
        this->declare_parameter<double>("j1_ticks_max", j1_ticks_max);

        // Joint 4 limits (float / double)
        this->declare_parameter<double>("j4_deg_min",   j4_deg_min);
        this->declare_parameter<double>("j4_deg_max",   j4_deg_max);
        this->declare_parameter<double>("j4_ticks_min", j4_ticks_min);
        this->declare_parameter<double>("j4_ticks_max", j4_ticks_max);

        // Push joint-limit params into the globals used by callbacks/helpers
        j1_deg_min    = this->get_parameter("j1_deg_min").as_double();
        j1_deg_max    = this->get_parameter("j1_deg_max").as_double();
        j1_ticks_min  = this->get_parameter("j1_ticks_min").as_double();
        j1_ticks_max  = this->get_parameter("j1_ticks_max").as_double();
        j4_deg_min    = this->get_parameter("j4_deg_min").as_double();
        j4_deg_max    = this->get_parameter("j4_deg_max").as_double();
        j4_ticks_min  = this->get_parameter("j4_ticks_min").as_double();
        j4_ticks_max  = this->get_parameter("j4_ticks_max").as_double();

        RCLCPP_INFO(this->get_logger(),
            "J1 limits: deg=[%.1f, %.1f] ticks=[%.1f, %.1f]",
            (double)j1_deg_min, (double)j1_deg_max,
            (double)j1_ticks_min, (double)j1_ticks_max);
        RCLCPP_INFO(this->get_logger(),
            "J4 limits: deg=[%.1f, %.1f] ticks=[%.1f, %.1f]",
            (double)j4_deg_min, (double)j4_deg_max,
            (double)j4_ticks_min, (double)j4_ticks_max);

        j1_sync_velocity_ = 9.0;
        j4_sync_velocity_ = 30.0;

        // Drive
        velocity_sub_ = this->create_subscription<Twist>(
            "/cmd_vel", 10, std::bind(&ArmDriveNode::velocity_callback, this, _1));

        // Joint angles
        joint1_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_teleop/joint1", 10, std::bind(&ArmDriveNode::joint1_callback, this, _1));
        joint4_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_teleop/joint4", 10, std::bind(&ArmDriveNode::joint4_callback, this, _1));

        // Synchronized velocities
        j1_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_sync/joint1_velocity", 10, std::bind(&ArmDriveNode::j1_vel_callback, this, _1));
        j4_vel_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_sync/joint4_velocity", 10, std::bind(&ArmDriveNode::j4_vel_callback, this, _1));

        // Gripper
        gripper_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_teleop/gripper", 10, std::bind(&ArmDriveNode::gripper_callback, this, _1));

        // Linear actuator
        linear_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/arm_teleop/linear_actuator", 10, std::bind(&ArmDriveNode::linear_callback, this, _1));

        // Drive feedback
        pub_left_vel_ = this->create_publisher<std_msgs::msg::Float64>("left_targetVelocity", 10);
        pub_right_vel_ = this->create_publisher<std_msgs::msg::Float64>("right_targetVelocity", 10);

        // Joint feedback in DEGREES
        pub_j1_deg_ = this->create_publisher<std_msgs::msg::Float64>("/arm_feedback/joint1_deg", 10);
        pub_j4_deg_ = this->create_publisher<std_msgs::msg::Float64>("/arm_feedback/joint4_deg", 10);

        // Joint feedback in raw ticks
        pub_j1_ticks_ = this->create_publisher<std_msgs::msg::Float64>("/arm_feedback/joint1_ticks", 10);
        pub_j4_ticks_ = this->create_publisher<std_msgs::msg::Float64>("/arm_feedback/joint4_ticks", 10);

        // Feedback timer (20 Hz)
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArmDriveNode::publish_feedback, this));

        RCLCPP_INFO(this->get_logger(),
            "CTRE node ready (J1=ID7, J4=ID15, Gripper=ID14, Linear=ID17, Drive)");
    }

private:
    void j1_vel_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double vel = std::max(1.0, std::min(msg->data, 9.0));
        j1_sync_velocity_.store(vel);
        srxArm1.ConfigMotionCruiseVelocity(static_cast<int>(vel), 0);
    }

    void j4_vel_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double vel = std::max(3.0, std::min(msg->data, 30.0));
        j4_sync_velocity_.store(vel);
        srxArm5.ConfigMotionCruiseVelocity(static_cast<int>(vel), 0);
    }

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

    // Joint 1 — using EXACT mapping from working code
    void joint1_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double angle = msg->data+phys_error;
        if (angle < j1_deg_min || angle > j1_deg_max) {
            RCLCPP_WARN(this->get_logger(), "J1 angle %.1f out of range [%.1f, %.1f]",
                        angle, (double)j1_deg_min, (double)j1_deg_max);
            return;
        }
        ctre::phoenix::unmanaged::FeedEnable(10000);
        // From working code: my_map(-msg->data, -85, 90, 1860, 0)
        double target = my_map(angle, j1_deg_min, j1_deg_max, j1_ticks_min, j1_ticks_max);
        srxArm1.Set(ControlMode::MotionMagic, target);
    }

    // Joint 4 — using EXACT mapping from working code
    void joint4_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double angle = msg->data;
        if (angle < j4_deg_min || angle > j4_deg_max) {
            RCLCPP_WARN(this->get_logger(), "J4 angle %.1f out of range [%.1f, %.1f]",
                        angle, (double)j4_deg_min, (double)j4_deg_max);
            return;
        }
        ctre::phoenix::unmanaged::FeedEnable(10000);
        // From working code: my_map(-msg->data, -150, 150, 205, 3618)
        double target = my_map(-angle, j4_deg_min, j4_deg_max, j4_ticks_min, j4_ticks_max);
        srxArm5.Set(ControlMode::MotionMagic, target);
    }

    void gripper_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        ctre::phoenix::unmanaged::FeedEnable(10000);
        srxGripper.Set(ControlMode::PercentOutput, msg->data);
    }

    void linear_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        ctre::phoenix::unmanaged::FeedEnable(10000);
        srxLineal.Set(ControlMode::PercentOutput, msg->data);
    }

    void publish_feedback() {
        double j1_ticks = static_cast<double>(srxArm1.GetSelectedSensorPosition());
        double j4_ticks = static_cast<double>(srxArm5.GetSelectedSensorPosition());

        std_msgs::msg::Float64 msg;

        // Raw ticks
        msg.data = j1_ticks;
        pub_j1_ticks_->publish(msg);
        msg.data = j4_ticks;
        pub_j4_ticks_->publish(msg);

        // Degrees
        msg.data = ticks_to_deg_j1(j1_ticks);
        pub_j1_deg_->publish(msg);
        msg.data = ticks_to_deg_j4(j4_ticks);
        pub_j4_deg_->publish(msg);
    }

    std::atomic<double> j1_sync_velocity_;
    std::atomic<double> j4_sync_velocity_;

    rclcpp::Subscription<Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint4_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr j1_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr j4_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr linear_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_j1_deg_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_j4_deg_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_j1_ticks_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_j4_ticks_;

    rclcpp::TimerBase::SharedPtr feedback_timer_;
};

// ==========================================================================
// Main — same pattern as working code
// ==========================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (start_config) {
        // Read CAN-ID parameters from a temporary node before constructing motor objects
        auto param_node = std::make_shared<rclcpp::Node>("arm_drive_ctre_params");
        can_id_arm1    = param_node->declare_parameter<int>("can_id_arm1",    can_id_arm1);
        can_id_arm5    = param_node->declare_parameter<int>("can_id_arm5",    can_id_arm5);
        can_id_gripper = param_node->declare_parameter<int>("can_id_gripper", can_id_gripper);
        can_id_lineal  = param_node->declare_parameter<int>("can_id_lineal",  can_id_lineal);
        can_id_fl      = param_node->declare_parameter<int>("can_id_fl",      can_id_fl);
        can_id_fr      = param_node->declare_parameter<int>("can_id_fr",      can_id_fr);
        can_id_bl      = param_node->declare_parameter<int>("can_id_bl",      can_id_bl);
        can_id_br      = param_node->declare_parameter<int>("can_id_br",      can_id_br);

        std::cout << "CAN IDs -> arm1=" << can_id_arm1
                  << " arm5=" << can_id_arm5
                  << " gripper=" << can_id_gripper
                  << " lineal=" << can_id_lineal
                  << " | drive FL=" << can_id_fl
                  << " FR=" << can_id_fr
                  << " BL=" << can_id_bl
                  << " BR=" << can_id_br << std::endl;

        // Construct motor objects with parameterized IDs
        srxArm1_ptr       = std::make_unique<TalonSRX>(can_id_arm1);
        srxArm5_ptr       = std::make_unique<TalonSRX>(can_id_arm5);
        srxGripper_ptr    = std::make_unique<TalonSRX>(can_id_gripper);
        srxLineal_ptr     = std::make_unique<TalonSRX>(can_id_lineal);
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
    int fw = srxArm5.GetFirmwareVersion();
    if (fw == -1) {
        std::cerr << "⚠️  WARNING: srxArm5 (ID " << can_id_arm5 << ") not responding on CAN bus!" << std::endl;
    } else {
        std::cout << "✅ srxArm5 (ID " << can_id_arm5 << ") firmware: " << fw << std::endl;
    }

    rclcpp::spin(std::make_shared<ArmDriveNode>());
    rclcpp::shutdown();
    return 0;
}
