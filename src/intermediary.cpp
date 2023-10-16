#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using Twist = geometry_msgs::msg::Twist;


class IntermediateNode : public rclcpp::Node {

private: 

	 rclcpp::Subscription<Twist>::SharedPtr subscription_drive_teleop;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_angles;

    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_front_left;
    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_front_right;
    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_back_left;
    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_back_right;

        float frontLeftAngle  = 0.0;
        float frontRightAngle = 0.0;
        float backLeftAngle   = 0.0;
        float backRightAngle  = 0.0;
        Twist twist;


public:
        IntermediateNode() : Node("intermediate_node") {
        
        subscription_drive_teleop = this->create_subscription<Twist>("/cmd_vel", 10, std::bind(&IntermediateNode::driveTeleopCallback, this, _1));
        subscription_angles = this->create_subscription<std_msgs::msg::Float64>("/angulos", 10, std::bind(&IntermediateNode::anglesCallback, this, _1));

        publisher_front_left  = this->create_publisher<std_msgs::msg::Float64>("/swerve/front_left", 10);
        publisher_front_right = this->create_publisher<std_msgs::msg::Float64>("/swerve/front_right", 10);
        publisher_back_left   = this->create_publisher<std_msgs::msg::Float64>("/swerve/back_left", 10);
        publisher_back_right  = this->create_publisher<std_msgs::msg::Float64>("/swerve/back_right", 10);
    }

    	void driveTeleopCallback(const Twist::SharedPtr msg) {
        
            twist.linear.y  = msg->linear.y;
            twist.linear.x  = msg->linear.x;
            twist.angular.z = msg->angular.z;

            if (msg->linear.y != 0 and msg->angular.z == 0){
                
                frontLeftAngle  = 90.0;
                frontRightAngle = 90.0;
                backLeftAngle   = 90.0;
                backRightAngle  = 90.0;
                publish_angles();
            }
            if(msg->angular.z != 0 and msg->linear.y == 0){

                frontLeftAngle  = -45.0;
                frontRightAngle = 45.0;
                backLeftAngle   = 45.0;
                backRightAngle  = -45.0;
                publish_angles();
            }
    }

        void anglesCallback(const std_msgs::msg::Float64::SharedPtr msg){
        
            if(twist.linear.y == 0  and twist.angular.z == 0){

                frontLeftAngle  = static_cast<float>(msg->data);
                frontRightAngle = static_cast<float>(msg->data);
                backLeftAngle   = static_cast<float>(msg->data);
                backRightAngle  = static_cast<float>(msg->data);
                publish_angles();
            }

        }

    void publish_angles(){

            std_msgs::msg::Float64 frontLeftMsg; 
            frontLeftMsg.data = frontLeftAngle;
            publisher_front_left->publish(frontLeftMsg);

            std_msgs::msg::Float64 frontRightMsg;
            frontRightMsg.data = frontRightAngle;
            publisher_front_right->publish(frontRightMsg);

            std_msgs::msg::Float64 backLeftMsg;
            backLeftMsg.data = backLeftAngle;
            publisher_back_left->publish(backLeftMsg);

            std_msgs::msg::Float64 backRightMsg;
            backRightMsg.data = backRightAngle;
            publisher_back_right->publish(backRightMsg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntermediateNode>());
    rclcpp::shutdown();
    return 0;
    
}
