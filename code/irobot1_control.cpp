#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include "cortex_bridge/cortexSetOrigin.h"

using namespace std;
const float pi = 3.14159265;

int main(int argc, char **argv){
	
	//init ROS
	ros::init(argc,argv,"irobot1_control");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	tf::TransformListener listener;

	//init variables
	double reference_ang[3];
	reference_ang[0]=0.0;	//r
	reference_ang[1]=0.0;	//p
	reference_ang[2]=pi/2;	//y

	// reference position
	float reference_position[3];
	reference_position[0] = 1.0;  // x
	reference_position[1] = 0.0;  // y
	reference_position[2] = 0.0;  // z
	// error
	float position_error[3];
	position_error[0] = 0.0;  // x
	position_error[1] = 0.0;  // y
	position_error[2] = 0.0;  // z

	
	//Initialize PID controller
	double Output[3];
	double errSum[3], lastErr[3];
	double ITerm[3], lastInput[3];
	double kp=0.1,ki=0.1,kd=0.01;
	double Kp=0.1,Kd=0.01,Ki=0.1;
	int SampleTime = 1;//sec
	double lastTime=ros::Time::now().toSec();
	double outMin= -0.3, outMax=0.3;

	// create command publisher
	ros::Publisher irobot1_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	while ( ros::ok() ){
		tf::StampedTransform transform;
		try{
			//grab new transform
			listener.lookupTransform("/world", "/irobot1", ros::Time(0), transform);
			// displacement
			
			//position_error[0] = reference_position[0] - transform.getOrigin().x();  // x
			//position_error[1] = reference_position[1] - transform.getOrigin().y();  // y
			//position_error[2] = reference_position[2] - transform.getOrigin().z();  // z
			
			//position_error[0] = transform.getOrigin().x();  // x
			//position_error[1] = transform.getOrigin().y();  // y
			//position_error[2] = transform.getOrigin().z();  // z
			
			// angular 
			float current_yaw = tf::getYaw(transform.getRotation())*180/pi;
			double r,p,y;
			transform.getBasis().getRPY(r,p,y);
			//tf::Quaternion  quat; 
			//quat = transform.getRotation();
			
			//tf::Matrix3x3 m(quat);
			//m.getRPY(r,p,y);
			double ang_error = reference_ang[2] - y;

			double distance_to_goal = sqrt(pow(position_error[0],2)+pow(position_error[1],2));

			//PID controller
			int now = ros::Time::now().toSec();
			int timeChange = now - lastTime;

			if (timeChange>=SampleTime){
				//compute error
				position_error[0] = reference_position[0] - transform.getOrigin().x();  // x
				position_error[1] = reference_position[1] - transform.getOrigin().y();  // y
				ang_error = reference_ang[2] - y; //theta
	
				errSum[0] += ki * position_error[0];
				errSum[1] += ki * position_error[1];
				errSum[2] += ki * ang_error;
				for (int i = 0; i < 3; i++)
					if (errSum[i] > outMax)
						errSum[i] = outMax;
					else 
						if (errSum[i] < outMin)
							errSum[i] = outMin;

				double dInput[3];
				dInput[0] =  transform.getOrigin().x() - lastInput[0];
				dInput[1] =  transform.getOrigin().y() - lastInput[1];
				dInput[2] =  y - lastInput[2];

				Output[0]=kp * position_error[0] + errSum[0] -kd * dInput[0];
				Output[1]=kp * position_error[1] + errSum[1] -kd * dInput[1];
				Output[2]=kp * ang_error + errSum[2] -kd * dInput[2];
				for (int i = 0; i < 3; i++)
					if (Output[i] > outMax)
						Output[i] = outMax;
					else 
						if (Output[i] < outMin)
							Output[i] = outMin;


				lastInput[0] = transform.getOrigin().x();
				lastInput[1] = transform.getOrigin().y(); 
				lastInput[2] = y;

				lastTime = now;

			}
			kp = Kp;
			ki = Ki * SampleTime;
			kd = Kd / SampleTime;
			//end of PID controller

			

			//velocity command
			geometry_msgs::Twist vel;
			if (distance_to_goal < 0.1) {
				vel.linear.x = 0;
				//ROS_INFO_STREAM("Stop because of position error: " << position_error[0]);
				//vel.linear.y = 0;
			}
			else{
				vel.linear.x = Output[0];
				//vel.linear.y = Output[1];	
			}
			if (ang_error < 0.1){
				vel.angular.z = 0;
				//ROS_INFO_STREAM("Stop because of angular error: " << ang_error);
			}
			else
				vel.angular.z = Output[2];

			irobot1_pub.publish(vel);
			
			//cout << "Distance to goal in x direction: " << position_error[0] << "\n";
			//cout << "Distance to goal in y direction: " << position_error[1] << "\n";
			cout << "Distance to goal : " << distance_to_goal << "\n";
			cout << "Different in yaw: " << ang_error <<"\n";
			cout << "V_x: " << Output[0] << "\n";
			cout << "V_angular: " << Output[2] << "\n";

			cout << "Current x: " << transform.getOrigin().x() << "\n";
			cout << "Current y: " << transform.getOrigin().y() << "\n";
			cout << "Current yaw: " << current_yaw << "\n";
			/*
			//print
			ROS_INFO_STREAM("Current x: " << transform.getOrigin().x());
			ROS_INFO_STREAM("Current y: " << transform.getOrigin().y());
			ROS_INFO_STREAM("Current theta in degree: " << current_yaw);
			ROS_INFO_STREAM("Current theta in degree from quaternion: " << y*180/pi);
			*/
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		//sleep
		rate.sleep();
	}

	return 0;
}

