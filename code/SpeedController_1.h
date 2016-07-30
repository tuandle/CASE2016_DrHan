#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

class SpeedController {
	public:
		SpeedController(ros::NodeHandle &nh);
		~SpeedController();

		void GetToGoal(double *goal_x, double *goal_y,double *goal_yaw);
		//void ComputeError();
		//void setKpKiKd();

	private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		tf::TransformListener listener_;

};