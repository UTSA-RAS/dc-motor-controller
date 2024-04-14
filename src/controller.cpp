#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "rover-controller/motorFeedback.msg"
#include <iostream>
#include <vector>

// PD Controller Gains
float Kp = 0.1;
float Kd = 0.5;

// State Variables
float target_velocity;
float measured_velocity;
float error;			// Controller error
float output;			// Instructions to ESP32
ros::Time measured_time;	// Feedback velocity time stamp
ros::Time last_time;		// Previous velocity time stamp for derivative

// Publisher for output
ros::Publisher motor_cmd_pub;

void targetVelocityCallback(const std_msgs::Float32::ConstPtr& msg) {
	target_velocity = msg->data;
}

void PD_Controller() {
	
}
