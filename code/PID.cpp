#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <PID/PID.h>


PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd){
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	PID::SetOutputLimit(-0.3,0.3);
	SampleTime = 0.5;
	PID::SetTunings(Kp, Ki, Kd);
	lastTime = ros::Time::now().toSec() - SampleTime;
}
PID::~PID(){};

bool PID::Compute(){
	double now = ros::Time::now().toSec();
	double timeChange = now - lastTime;
	if (timeChange >= SampleTime){
		double input = *myInput;
		double error = *mySetpoint - input;
		Iterm += ki * error;
		if (Iterm > outMax)
			Iterm = outMax;
		else 
			if (Iterm < outMin) 
				Iterm = outMin;
		double dInput = input - lastInput;

		double output = kp * error + Iterm - kd * dInput;
		if (output > outMax)
			output = outMax;
		else
			if (output < outMin)
				output = outMin;
		*myOutput = output;

		lastInput = input;
		lastTime = now;
		return true;
	}
	else
		return true;
}

void PID::SetTunings(double Kp, double Ki, double Kd){
	if (Kp < 0 || Ki < 0 || Kd < 0) return;
	kp = Kp;
	ki = Ki * SampleTime;
	kd = Kd / SampleTime;
}

void PID::SetSampleTime(int NewSampleTime){
	if (NewSampleTime > 0){
		double ratio = NewSampleTime / SampleTime;
		ki *= ratio;
		kd /= ratio;
		SampleTime = NewSampleTime;
	}
}

void PID::SetOutputLimit(double Min, double Max){
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;
	if (*myOutput > outMax)
		*myOutput = outMax;
	else
		if (*myOutput < outMin)
			*myOutput = outMin;
	if (Iterm > outMax)
		Iterm = outMax;
	else
		if (Iterm < outMin)
			Iterm = outMin;
}

void PID::Initialize(){
	Iterm = *myOutput;
	lastInput = *myInput;
	if (Iterm > outMax)
		Iterm = outMax;
	else
		if (Iterm < outMin)
			Iterm = outMin;
}