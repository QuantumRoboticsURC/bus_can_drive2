#define Phoenix_No_WPI // remove WPI dependencies
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sstream>
#include <signal.h>
#include <unistd.h>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using Twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;

bool start_config = true;

/* make some talons for arm train */
TalonSRX srxArm1(11);
TalonSRX srxSwrvFL(12);
TalonSRX srxSwrvFr(16);
TalonSRX srxSwrvBl(13);
TalonSRX srxArm5(15); //Gripper
TalonSRX srxArm6(21); //Prismatico
TalonSRX srxSwrvBr(14);

/* make some talons for drive train */
TalonFX talFrontLeft(0);
TalonFX talFrontRight(2);
TalonFX talBackLeft(1);
TalonFX talBackRight(3);


double my_map(double x, double in_min, double in_max, double out_min, double out_max){
        double targetPos = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return targetPos;
}


void initDrive()
{
/* Factory default hardware to prevent unexpected behavior */
srxArm1.ConfigFactoryDefault();
srxSwrvFL.ConfigFactoryDefault();
srxSwrvFr.ConfigFactoryDefault();
srxSwrvBl.ConfigFactoryDefault();
srxArm5.ConfigFactoryDefault();
srxArm6.ConfigFactoryDefault();

srxSwrvBr.ConfigFactoryDefault();


srxArm1.SetInverted(false);
srxSwrvFL.SetInverted(false);
srxSwrvFr.SetInverted(true);
srxSwrvBl.SetInverted(true);

srxSwrvBr.SetInverted(false);

srxArm1.SetSensorPhase(false);
srxSwrvFL.SetSensorPhase(false);
srxSwrvFr.SetSensorPhase(true);
srxSwrvBl.SetSensorPhase(true);

srxSwrvBr.SetSensorPhase(false);


/* choose the sensor and sensor direction */
srxArm1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
srxSwrvFL.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
srxSwrvFr.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
srxSwrvBl.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

srxSwrvBr.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);

//srxArm1.SetSelectedSensorPosition(offset1, 0, 10);


/* Set the peak and nominal outputs */
srxArm1.ConfigNominalOutputForward(0, 10);
srxArm1.ConfigNominalOutputReverse(0, 10);
srxArm1.ConfigPeakOutputForward(1, 10);
srxArm1.ConfigPeakOutputReverse(-1, 10);

srxSwrvFL.ConfigNominalOutputForward(0, 10);
srxSwrvFL.ConfigNominalOutputReverse(0, 10);
srxSwrvFL.ConfigPeakOutputForward(1, 10);
srxSwrvFL.ConfigPeakOutputReverse(-1, 10);

srxSwrvFr.ConfigNominalOutputForward(0, 10);
srxSwrvFr.ConfigNominalOutputReverse(0, 10);
srxSwrvFr.ConfigPeakOutputForward(1, 10);
srxSwrvFr.ConfigPeakOutputReverse(-1, 10);

srxSwrvBl.ConfigNominalOutputForward(0, 10);
srxSwrvBl.ConfigNominalOutputReverse(0, 10);
srxSwrvBl.ConfigPeakOutputForward(1, 10);
srxSwrvBl.ConfigPeakOutputReverse(-1, 10);

srxArm5.ConfigNominalOutputForward(0, 10);
srxArm5.ConfigNominalOutputReverse(0, 10);
srxArm5.ConfigPeakOutputForward(1, 10);
srxArm5.ConfigPeakOutputReverse(-1, 10);

srxArm6.ConfigNominalOutputForward(0, 10);
srxArm6.ConfigNominalOutputReverse(0, 10);
srxArm6.ConfigPeakOutputForward(1, 10);
srxArm6.ConfigPeakOutputReverse(-1, 10);


srxSwrvBr.ConfigNominalOutputForward(0, 10);
srxSwrvBr.ConfigNominalOutputReverse(0, 10);
srxSwrvBr.ConfigPeakOutputForward(1, 10);
srxSwrvBr.ConfigPeakOutputReverse(-1, 10);

/* set closed loop gains in slot0, editar valor de en medio */ 
/* Set acceleration and vcruise velocity - see documentation */
srxArm1.Config_kF(0, 82.25, 10);
srxArm1.Config_kP(0, 5.52972973, 10); 
srxArm1.Config_kI(0, 0.001, 10);
srxArm1.Config_kD(0, 55.2972973, 10);
srxArm1.ConfigMotionCruiseVelocity(9, 10);
srxArm1.ConfigMotionAcceleration(9, 10);

/* set closed loop gains in slot0, editar valor de en medio */ 
/* Set acceleration and vcruise velocity - see documentation */
srxSwrvFL.Config_kF(0, 46.5, 10);
srxSwrvFL.Config_kP(0, 1.794736842, 10); 
srxSwrvFL.Config_kI(0, 0.001, 10);
srxSwrvFL.Config_kD(0, 17.94736842, 10);
srxSwrvFL.ConfigMotionCruiseVelocity(17, 10);
srxSwrvFL.ConfigMotionAcceleration(16.5, 10);

/* set closed loop gains in slot0, editar valor de en medio */ 
/* Set acceleration and vcruise velocity - see documentation */
srxSwrvFr.Config_kF(0, 46.5, 10);
srxSwrvFr.Config_kP(0, 1.794736842, 10); 
srxSwrvFr.Config_kI(0, 0.001, 10);
srxSwrvFr.Config_kD(0, 17.94736842, 10);
srxSwrvFr.ConfigMotionCruiseVelocity(17, 10);
srxSwrvFr.ConfigMotionAcceleration(16.5, 10);

/* set closed loop gains in slot0, editar valor de en medio */ 
/* Set acceleration and vcruise velocity - see documentation */
srxSwrvBl.Config_kF(0, 34.1, 10);
srxSwrvBl.Config_kP(0, 6.2, 10); 
srxSwrvBl.Config_kI(0, 0.001, 10);
srxSwrvBl.Config_kD(0, 62.0, 10);
srxSwrvBl.ConfigMotionCruiseVelocity(23, 10);
srxSwrvBl.ConfigMotionAcceleration(22.5, 10);


srxSwrvBr.Config_kF(0, 34.1, 0);
srxSwrvBr.Config_kP(0, 9.742857143, 0); 
srxSwrvBr.Config_kI(0, 0.001, 0);
srxSwrvBr.Config_kD(0, 97.42857143, 0);
srxSwrvBr.ConfigMotionCruiseVelocity(23, 0);
srxSwrvBr.ConfigMotionAcceleration(22.5, 0);


/* Factory default hardware to prevent unexpected behavior */
talFrontLeft.ConfigFactoryDefault();
talFrontRight.ConfigFactoryDefault();
talBackLeft.ConfigFactoryDefault();
talBackRight.ConfigFactoryDefault();



/* Configure Sensor Source for Pirmary PID */
talFrontLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
talFrontRight.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
talBackLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
talBackRight.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

/**
* Configure Talon SRX Output and Sensor direction accordingly
* Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
* Phase sensor to have positive increment when driving Talon Forward (Green LED)
*/
talFrontLeft.SetSensorPhase(false);
talFrontLeft.SetInverted(false);
talFrontRight.SetSensorPhase(true);
talFrontRight.SetInverted(true);
talBackLeft.SetSensorPhase(false);
talBackLeft.SetInverted(false);
talBackRight.SetSensorPhase(true);
talBackRight.SetInverted(true);

/* Set the peak and nominal outputs */
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

/* Set Motion Magic gains in slot0 - see documentation */
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

/* simple wrapper for code cleanup */
void sleepApp(int ms)
{
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

//Esta es la clase que inicializa lo necesario para ROS 2
class Drive_can: public rclcpp::Node
{
    private:
        bool start_config = true;
        rclcpp::Subscription<Twist>::SharedPtr velocity_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint1_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_left_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_left_;
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_right_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr centrifuge_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr prism_;
		
		bool front_left_checker;
		bool front_right_checker;
		bool back_left_checker;
		bool back_right_checker;
        
        
        
    public:
    	    Drive_can(): Node("drive_can"){
    	    velocity_ =this->create_subscription<Twist>("/cmd_vel", 10, std::bind(&Drive_can::velocityCallback, this, _1));
    	    joint1_ =this->create_subscription<std_msgs::msg::Float64>("/arm_teleop/joint1", 10, std::bind(&Drive_can::joint1Callback, this, _1));
    	    front_left_ =this->create_subscription<std_msgs::msg::Float64>("/swerve/front_left", 10, std::bind(&Drive_can::frontleftCallback, this, _1));
    	    front_right_ =this->create_subscription<std_msgs::msg::Float64>("/swerve/front_right", 10, std::bind(&Drive_can::frontrightCallback, this, _1));
    	    back_left_ =this->create_subscription<std_msgs::msg::Float64>("/swerve/back_left", 10, std::bind(&Drive_can::backleftCallback, this, _1));
	        back_right_ =this->create_subscription<std_msgs::msg::Float64>("/swerve/back_right", 10, std::bind(&Drive_can::backrightCallback, this, _1));
    	    //centrifuge_ =this->create_subscription<std_msgs::msg::Float64>("/swerve/centrifuge", 10, std::bind(&Drive_can::centrifugeCallback, this, _1));
    	    //prism_ =this->create_subscription<std_msgs::msg::Float64>("/arm_teleop_prism", 10, std::bind(&Drive_can::prism_callback, this, std::placeholders::_1));
    	    front_left_checker=false;
			front_right_checker=false;
			back_left_checker=false;
			back_right_checker=false;
    	    
			}

    
	void velocityCallback(const Twist::SharedPtr msg) const{
		bool check =front_left_checker and front_right_checker and back_left_checker and back_right_checker;
		std::cout<<check<<"\n";
		if(front_left_checker and front_right_checker and back_left_checker and back_right_checker){
			/* Magic velocity */
			/* 2048 units/rev * 1 Rotations in either direction */
			double rght = (msg->linear.x + msg->angular.z);
			double left = (msg->linear.x - msg->angular.z);
			if ((rght == 0.0) and (left == 0)){
				rght = msg->linear.y;
				left = msg->linear.y;
			}
            else if (msg->angular.z ==0){
                rght = -rght;
                left = -left;
			}
			double left_targetVelocity=left*6000*2048/600;//left*velocidad que otorga el motor*resolucion enc$
			double right_targetVelocity=rght*6000*2048/600;//left*velocidad que otorga el motor*resolucion en$

			ctre::phoenix::unmanaged::FeedEnable(5000);
			if (left == 0){
				talFrontLeft.Set (ControlMode::PercentOutput, left);
				talBackLeft.Set  (ControlMode::PercentOutput, left);
			}
			else if(msg->linear.y == 0){
				talFrontLeft.Set (ControlMode::Velocity, left_targetVelocity);
				talBackLeft.Set  (ControlMode::Velocity, left_targetVelocity);
			}
            else{
                talFrontLeft.Set (ControlMode::Velocity, -left_targetVelocity);
                talBackLeft.Set (ControlMode::Velocity, left_targetVelocity);
            }
			if (rght == 0){
				talFrontRight.Set (ControlMode::PercentOutput, rght);
				talBackRight.Set  (ControlMode::PercentOutput, rght);
			}
			else if(msg->linear.y == 0){
				talFrontRight.Set (ControlMode::Velocity, right_targetVelocity);
				talBackRight.Set  (ControlMode::Velocity, right_targetVelocity);
			}
            else{
				talFrontRight.Set (ControlMode::Velocity, -right_targetVelocity);
                talBackRight.Set (ControlMode::Velocity, right_targetVelocity); 
            }

			/*Get current Talon SRX motor output */
			std::stringstream sb;
			/* Prepare line to print */
			std::cout << "\tOutput%:" << talFrontLeft.GetMotorOutputPercent() << "\n";
			std::cout << "\tEncoder Velocity:" << talFrontLeft.GetSelectedSensorVelocity(0) << "\n";
			std::cout << "\tTeorical Velocity:" << left_targetVelocity << "\n\n";
		}else{
			std::cout << "\n\n \tSwerve is reaching the goal"<<"\n\n";
			talFrontLeft.Set (ControlMode::Velocity, 0);
            talBackLeft.Set (ControlMode::Velocity, 0);
			talFrontRight.Set (ControlMode::Velocity, 0);
            talBackRight.Set (ControlMode::Velocity, 0); 
		}
    }
	    
	void joint1Callback(const std_msgs::msg::Float64::SharedPtr msg) const{
		ctre::phoenix::unmanaged::FeedEnable(10000);  
		/* Motion Magic */     
		/*4096 ticks/rev in either direction */
		/* input should be a value from -110 to 110 deg */
		if (msg->data >= -90 && msg->data <= 180){
		//int offset1_deg = 50.2734375 + 180; //Cero at  center
		//double targetPos = (msg.data + offset1_deg) * 4096 / 360;
		double targetPos = my_map(msg->data, -90, 180, 191, 3263);
		srxArm1.Set(ControlMode::MotionMagic, targetPos);
		//srxArm1.Set(ControlMode::PercentOutput, targetPos);

		/* Prepare line to print */
		std::stringstream sb;
		int dif_error = srxArm1.GetSelectedSensorPosition() - targetPos;
		std::cout << "\tAxis 2 Output\n";
		//double sensorPosition = srxArm1.GetSelectedSensorPosition() / 4096 / 24 * 360;
		//double targetPos = msg->data * 4096 * 24 / 360;
		std::cout << "\tTarget Position Deg: " << msg->data << " degrees\n";
		std::cout << "\tEncoder Position Ticks:" << srxArm1.GetSelectedSensorPosition() << "ticks\n";
		std::cout << "\tEncoder Error:" << dif_error << "\n";
		std::cout << "\tEncoder Velocity:" << srxArm1.GetSelectedSensorVelocity(0) << "\n\n";
		}
		else{
		std::stringstream sb;
		std::cout << "\tMust be a value from -110 to 110 degrees\n";
		}
	}
	// Motor del swerve izquierdo adelante (antes era el joint 2 del brazo)
	void frontleftCallback(const std_msgs::msg::Float64::SharedPtr msg) {
		ctre::phoenix::unmanaged::FeedEnable(30000);
		if (msg->data >= -90 && msg->data <= 90){
		//    int offset2_deg = 33.3984375; //grados, cero horizontal
		//  double targetPos = (msg.data + offset2_deg) * 4096 / 360;
		// double targetPos = my_map(msg.data, 10, 161, 510.77, 2228.8);
		double targetPos = my_map(msg->data,-90,90, 2063, 4111);  
		srxSwrvFL.Set(ControlMode::MotionMagic, targetPos);
			//srxArm1.Set(ControlMode::PercentOutput, targetPos);

			/* Prepare line to print */
			std::stringstream sb;
			int dif_error = srxSwrvFL.GetSelectedSensorPosition() - targetPos;
			front_left_checker=dif_error<170.66 and dif_error>-170.66;
			std::cout << "\tFront Left Checker Output\n";
			//double sensorPosition = srxArm1.GetSelectedSensorPosition() / 4096 / 24 * 360;
			//double targetPos = msg.data * 4096 * 24 / 360;
			std::cout << "\tTarget Position Deg: " << msg->data << " degrees\n";
			std::cout << "\tEncoder Position Ticks:" << srxSwrvFL.GetSelectedSensorPosition() << "ticks\n";
			std::cout << "\tEncoder Error:" << dif_error << "\n";
			std::cout << "\tEncoder Velocity:" << srxSwrvFL.GetSelectedSensorVelocity(0) << "\n";
			std::cout << "\tChecker: "<< front_left_checker<< "\n\n";
		}
		else{
			std::stringstream sb;
			std::cout << "\tMust be a value from -90 to 90 degrees\n";
		}
	}
	// Motor del swerve derecho adelante (antes era el joint3 del brazo)
	void frontrightCallback(const std_msgs::msg::Float64::SharedPtr msg) {
		ctre::phoenix::unmanaged::FeedEnable(10000);
		if (msg->data >= -90 && msg->data <= 90){
			//double x = msg->data;
			//int in_min = 0;
			//int in_max = -180;
			//int out_min = 3563;
			//int out_max = 1515;
			//double targetPos = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		double targetPos = my_map(msg->data,-90 , 90, 1005, 3053);
			srxSwrvFr.Set(ControlMode::MotionMagic, targetPos);
			//srxArm1.Set(ControlMode::PercentOutput, targetPos);

			/* Prepare line to print */
			std::stringstream sb;
			int dif_error = srxSwrvFr.GetSelectedSensorPosition() - targetPos;
			front_right_checker=dif_error<170.66 and dif_error>-170.66;
			std::cout << "\tFront Right Swerve Output\n";
			//double sensorPosition = srxArm1.GetSelectedSensorPosition() / 4096 / 24 * 360;
			//double targetPos = msg.data * 4096 * 24 / 360;
			std::cout << "\tTarget Position Deg: " << msg->data << " degrees\n";
			std::cout << "\tEncoder Position Ticks:" << srxSwrvFr.GetSelectedSensorPosition() << "ticks\n";
			std::cout << "\tEncoder Error:" << dif_error << "\n";
			std::cout << "\tEncoder Velocity:" << srxSwrvFr.GetSelectedSensorVelocity(0) << "\n";
			std::cout << "\tChecker: "<< front_right_checker<< "\n\n";
		}
		else{
			std::stringstream sb;
			std::cout << "\tMust be a value from -90 to 90 degrees\n";
		}
	}
	//Motor del swerve izquierdo atras (antes era el joint 4 del brazo)
	void backleftCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        ctre::phoenix::unmanaged::FeedEnable(10000); 
        if (msg->data >= -90 && msg->data <= 90){
        //double x = msg.data;
        //int in_min = -135;
        //int in_max = 90;
        //int out_min = 457;
        //int out_max = 3017;
        //double targetPos = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        double targetPos =my_map(msg->data,-90,90,824,2872);
        srxSwrvBl.Set(ControlMode::MotionMagic, targetPos);
        //srxSwrvBl.Set(ControlMode::PercentOutput, targetPos);

        /* Prepare line to print */
        std::stringstream sb;
        int dif_error = srxSwrvBl.GetSelectedSensorPosition() - targetPos;
		back_left_checker=dif_error<170.66 and dif_error>-170.66;
        std::cout << "\tBack Left Swerve Output\n";
        //double sensorPosition = srxArm1.GetSelectedSensorPosition() / 4096 / 24 * 360;
        //double targetPos = msg.data * 4096 * 24 / 360;
        std::cout << "\tTarget Position Deg: " << msg->data << " degrees\n";
        std::cout << "\tEncoder Position Ticks:" << srxSwrvBl.GetSelectedSensorPosition() << "ticks\n";
        std::cout << "\tEncoder Error:" << dif_error << "\n";
        std::cout << "\tEncoder Velocity:" << srxSwrvBl.GetSelectedSensorVelocity(0) << "\n";
		std::cout << "\tChecker: "<< back_left_checker<< "\n\n";
        }
        else{
            std::stringstream sb;
            std::cout << "\tMust be a value from -135 to 90 degrees\n";
        }
	}

    //Motor del swerve derecho atras (antes era el joint2 del brazo del laboratorio) 
	void backrightCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    	ctre::phoenix::unmanaged::FeedEnable(10000);  
    	if (msg->data >= -90 && msg->data <= 90){
        double x = msg->data;
        int in_min = -90;
        int in_max = 90;
        int out_min = 1420;
        int out_max = 3468;
        double targetPos = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        srxSwrvBr.Set(ControlMode::MotionMagic, targetPos);
        //srxSwrvBr.Set(ControlMode::PercentOutput, msg.data);
        /* Prepare line to print */
        std::stringstream sb;
        int dif_error = srxSwrvBr.GetSelectedSensorPosition() - targetPos;
		back_right_checker=dif_error<170.66 and dif_error>-170.66;
        std::cout << "\tBack Right Swerve\n";
        //double sensorPosition = srxArm1.GetSelectedSensorPosition() / 4096 / 24 * 360;
        //double targetPos = msg.data * 4096 * 24 / 360;
        std::cout << "\tTarget Position Deg: " << msg->data << " degrees\n";
        std::cout << "\tEncoder Position Ticks:" << srxSwrvBr.GetSelectedSensorPosition() << "ticks\n";
        std::cout << "\tEncoder Error:" << dif_error << "\n";
        std::cout << "\tEncoder Velocity:" << srxSwrvBr.GetSelectedSensorVelocity(0) << "\n";
		std::cout << "\tChecker: "<< back_right_checker<< "\n\n";
    	}
    	else{
        std::stringstream sb;
        std::cout << "\tMust be a value from -90 to 90 degrees\n";
    	}
	}	


};



int main(int argc, char **argv){
    bool start_config = true;
    if (start_config==true){
        std::string interface;
        interface = "can0";
        ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
        initDrive();
        start_config=false;
	}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Drive_can>());
	rclcpp::shutdown();
    return 0;
}
