#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <SpeedController/SpeedController.h>
#include <PID/PID.h>
using namespace std;

int main(int argc, char** argv){

	//Initialize ROS
	ros::init(argc,argv,"irobot1_control");
	ros::NodeHandle nh;
	SpeedController move(nh);
	//End of initializing ROS

	//Feed goal 
	/*
	double v_x = 0.2, v_angular = 0.2;
	bool done = false;
	move.GetToGoal(v_x,v_angular,done);
	/*
	/*
	bool done = false;
	int width = 50;
	double vel[2][width];
	ifstream fin;
	fin.open("ir_1.dat");
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < width; j++)
			fin >> vel[i][j];	
	fin.close();
	//End of feeding goal

	//Control command
	for (int i = 0; i < width; i++ ){
		double v_x = vel[0][i];
		double v_angular = vel[1][i];
		move.GetToGoal(v_x,v_angular,done);	
	}
	done = true;
	move.GetToGoal(0.0,0.0,done); // redundancy ?! 	
	//End of control command
	*/
	move.GetToGoal_pid(1,1,M_PI/2);

	
	return 0;
}