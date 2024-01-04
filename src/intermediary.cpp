//Librerias de ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" //velocidades angulares en ROS
#include "std_msgs/msg/float64.hpp" //tipo de float

using std::placeholders::_1; // reservar un espacio
using Twist = geometry_msgs::msg::Twist; // utilizar velocidad angular 


class IntermediateNode : public rclcpp::Node {

private: //(crear variables)

	 rclcpp::Subscription<Twist>::SharedPtr subscription_drive_teleop; // puntero (subscribirte a los mensajes que manda el dirve_teleop)
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_angles; // puntero (subscription)

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_front_left; // apuntadores especificos (crear los publicadores que mandan datos (drive_can))
    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_front_right;
    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_back_left;
    	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_back_right;

        float frontLeftAngle  = 0.0; // dandole valores a las variables a base de un FLoat
        float frontRightAngle = 0.0;
        float backLeftAngle   = 0.0;
        float backRightAngle  = 0.0;
        Twist twist; 


public:
        // se mada a llamar a la funcion Intermidiate node
        IntermediateNode() : Node("intermediate_node") {  
        // a esta variable se le ingresa el valor que va después del apuntador
        subscription_drive_teleop = this->create_subscription<Twist>("/cmd_vel", 10, std::bind(&IntermediateNode::driveTeleopCallback, this, _1));
        subscription_angles = this->create_subscription<std_msgs::msg::Float64>("/angle_swr", 10, std::bind(&IntermediateNode::anglesCallback, this, _1));

        publisher_front_left  = this->create_publisher<std_msgs::msg::Float64>("/swerve/front_left", 10);
        publisher_front_right = this->create_publisher<std_msgs::msg::Float64>("/swerve/front_right", 10);
        publisher_back_left   = this->create_publisher<std_msgs::msg::Float64>("/swerve/back_left", 10);
        publisher_back_right  = this->create_publisher<std_msgs::msg::Float64>("/swerve/back_right", 10);
    }
        // funciones que no regresan cosas pero si cumplen una funcion
    	void driveTeleopCallback(const Twist::SharedPtr msg) {

            // se le agrega el valor del mensaje a los linears
            twist.linear.y  = msg->linear.y;
            twist.linear.x  = msg->linear.x;
            twist.angular.z = msg->angular.z;
            
            // condiciones del control
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
        // condiciones del control
        void anglesCallback(const std_msgs::msg::Float64::SharedPtr msg){
        
            if(twist.linear.y == 0  and twist.angular.z == 0){

                frontLeftAngle  = static_cast<float>(msg->data);
                frontRightAngle = static_cast<float>(msg->data);
                backLeftAngle   = static_cast<float>(msg->data);
                backRightAngle  = static_cast<float>(msg->data);
                publish_angles();
            }

        }
    // PUBLICA ANGULOS
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
