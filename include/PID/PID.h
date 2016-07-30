#ifndef PID_h 
#define PID_h

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>


class PID {
	public:
		PID(double*,double*,double*,double,double,double); //Input, Output, Setpoint, Kp,Kd,Ki
		~PID();

		bool Compute();
		void SetOutputLimit(double, double);
		void SetSampleTime(int);
		void SetTunings(double,double,double);

	
	private:	
		void Initialize();
		double kp;
		double ki;
		double kd;
		double *myInput;
		double *myOutput;
		double *mySetpoint;
		double lastTime;
		double Iterm, lastInput;
		double SampleTime;
		double outMin, outMax;
		

};
#endif