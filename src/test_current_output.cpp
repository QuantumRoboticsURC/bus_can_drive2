#define Phoenix_No_WPI

#include <chrono>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/can/PlatformCAN.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class CurrentTestNode : public rclcpp::Node
{
public:
  CurrentTestNode()
  : Node("test_current_output_talon")
  {
    this->declare_parameter<int>("can_id", 7);
    this->declare_parameter<double>("publish_hz", 50.0);

    can_id_ = this->get_parameter("can_id").as_int();
    publish_hz_ = this->get_parameter("publish_hz").as_double();

    motor_ = std::make_unique<TalonSRX>(can_id_);

    std::string iface = "can0";
    ctre::phoenix::platform::can::PlatformCAN::SetCANInterface(iface.c_str());

    motor_->ConfigFactoryDefault();
    motor_->SetInverted(false);

    // Current limits for a safe bench test
    motor_->ConfigContinuousCurrentLimit(20, 10);
    motor_->ConfigPeakCurrentLimit(30, 10);
    motor_->ConfigPeakCurrentDuration(200, 10);
    motor_->EnableCurrentLimit(true);

    motor_->ConfigNominalOutputForward(0.0, 10);
    motor_->ConfigNominalOutputReverse(0.0, 10);
    motor_->ConfigPeakOutputForward(1.0, 10);
    motor_->ConfigPeakOutputReverse(-1.0, 10);

    setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "talon_current/setpoint_amp",
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg)
      {
        setpoint_amp_.store(msg->data);
        // Use current control mode for the actual test
        unmanaged::FeedEnable(100);
        motor_->Set(ControlMode::Current, msg->data);
      });

    feedback_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "talon_current/feedback_amp", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_hz_)),
      std::bind(&CurrentTestNode::publish_feedback, this));
  }

private:
  void publish_feedback()
  {
    std_msgs::msg::Float64 msg;
    msg.data = motor_->GetOutputCurrent();   // or GetStatorCurrent()
    feedback_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "setpoint=%.2f A, feedback=%.2f A",
      setpoint_amp_.load(), msg.data);
  }

  int can_id_;
  double publish_hz_;

  std::atomic<double> setpoint_amp_{0.0};

  std::unique_ptr<TalonSRX> motor_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentTestNode>());
  rclcpp::shutdown();
  return 0;
}