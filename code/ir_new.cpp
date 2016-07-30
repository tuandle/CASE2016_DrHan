#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <SpeedController/SpeedController.h>

int main(int argc, char** argv) {
	//Initialize ROS
	ros::init(argc,argv,"irobot1_control");
	ros::NodeHandle nh;
	SpeedController move(nh);
	//End of initializing ROS

	//Get current position
	
	//end of get current position

	move.Move_Han(1.3,0.5,0,0,0); //x0, y0, theta0, w0, v0 - initial conditions
}

