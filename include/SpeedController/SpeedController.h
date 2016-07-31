#ifndef SpeedController_h 
#define SpeedController_h

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nr3/nr3.h> 
#include <nr3/adapt.h> 

class SpeedController {
	public:
		SpeedController(ros::NodeHandle &nh);
		~SpeedController();

		void GetToGoal(double, double, bool);
		void GetToGoal_pid(double, double, double);
		double set_pose();
		double get_pose() const;
		double var_theta(double, double); // var_theta_const, integral omega
		double var_phi(double, double, double); //A, x, y
		//double psi(double, double); //theta, var_theta
		Doub satsm(double,double,double); // psi, const1, const2
		Doub mybump(double,double,double);
		Doub bumpf(double);
		double uctr(double,double,double,double,double); // var_phi, r, x, y, theta, linear velocity, psi
		double tau(double, double, double, double,double);//var_phi, r, x, y, theta, linear v, psi, satm, const1, const2
		double omega(double,double,double,double);//x, y, theta, linear v, r
		void Move_Han(double,double,double,double,double); // x0, y0, theta_0,v_0, omega_0,
		//void ComputeError();
		//void setKpKiKd();

	
	private:	
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		tf::TransformListener listener_, dummy_listener;
		double pose_x, pose_y, pose_theta;
		

};
#endif