#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <math.h>
#include <boost/numeric/odeint.hpp>
#include <geometry_msgs/Twist.h>
#include <SpeedController/SpeedController.h>
#include <PID/PID.h>
 
using namespace std;
 
SpeedController::SpeedController(ros::NodeHandle &nh){
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/irobot1/cmd_vel",1000);
}
 
SpeedController::~SpeedController(){}
 
void SpeedController::GetToGoal(double v_linear, double v_angular, bool done){
     
    ros::Rate rate(10);
    listener_.waitForTransform("/world", "/irobot1", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_;
 
    while (nh_.ok()){
        try{
            listener_.lookupTransform("/world", "/irobot1", ros::Time(0), transform_);//listen to current frame
            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x,current_y; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
             
            geometry_msgs::Twist vel_;
 
            /*
            double distance_to_goal = sqrt(pow(goal_x - current_x,2)+pow(goal_y - current_y,2)); // need to move ?
            double diff_yaw = goal_yaw - current_yaw; // need to turn?
            ROS_INFO_STREAM("Current x: " << current_x);
            ROS_INFO_STREAM("Current x: " << goal_x);
            ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
             
            //We need to move to our goal
            //Compute velocities
            if (abs(distance_to_goal) > 0.1){
                vel_.linear.x = 0.1;
                vel_.angular.z = 0.1;
            }
            else{
                vel_.linear.x = 0;
                vel_.angular.z = 0; 
            }
            //End of computing velocities
            */
            if (!done){
                vel_.linear.x = v_linear;
                vel_.angular.z = v_angular;
            }
            else{
                vel_.linear.x = 0;
                vel_.angular.z = 0; 
            }
            cmd_vel_pub_.publish(vel_); //publish velocities
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
 
     
void SpeedController::GetToGoal_pid(double x_goal, double y_goal, double yaw_goal){
    double Kp = 1, Ki = 0.01, Kd = 0.01;
    bool init = true;
    double SetPoint_linear = sqrt(pow(x_goal,2) + pow(y_goal,2)), SetPoint_angular = yaw_goal;
    double Input_linear, Output_linear, Input_angular;
    double Output_angular;
 
    PID pid_linear(&Input_linear, &Output_linear, &SetPoint_linear, Kp, Ki, Kd);
    pid_linear.SetOutputLimit(0,0.5);
     
    ros::Rate rate(100);
    listener_.waitForTransform("/world", "/irobot1", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_;
     
    while (nh_.ok()){
        try{
            listener_.lookupTransform("/world", "/irobot1", ros::Time(0), transform_);//listen to current frame
            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x,current_y; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
 
            Input_angular = current_yaw;
 
            geometry_msgs::Twist vel_;
             
            double distance_to_goal = pow((pow((x_goal - current_x),2)+pow((y_goal - current_y),2)),0.5); // need to move ?
            Input_linear = distance_to_goal;
             
            double diff_yaw = yaw_goal - current_yaw; // need to turn?
             
            ROS_INFO_STREAM("Current x: " << current_x);
            ROS_INFO_STREAM("Goal x: " << SetPoint_linear);
            ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
            ROS_INFO_STREAM("Yaw goal: " << yaw_goal);
            ROS_INFO_STREAM("Current yaw: " << current_yaw);
            ROS_INFO_STREAM("Diff in yaw: " << diff_yaw);
             
            if (abs(distance_to_goal) > 0.1){
                Output_angular = 10*(atan2((y_goal-current_y),(x_goal-current_x))-current_yaw);
                vel_.angular.z = Output_angular;
                cout << "Angular velocity: " << vel_.angular.z << "\n";
 
                pid_linear.Compute();
                //vel_.linear.x = 0.3;
                vel_.linear.x = Output_linear;
                cout << "Linear velocity: " << vel_.linear.x << "\n";
                //pid_angular.Compute();
                //vel_.angular.z = Output[1];
                 
            }
            else{
                //vel_.linear.x = 0;
                if (abs(diff_yaw) < 0.01){
                    vel_.angular.z = 0;
                }
            } 
 
            cmd_vel_pub_.publish(vel_); //publish velocities
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
 
double SpeedController::omega(double x, double y, double theta, double v){
    double r = sqrt(pow(x,2)+pow(y,2));
    return ((x*sin(theta) - y*cos(theta)) * v / pow(r,2)); 
}

double SpeedController::var_phi(double A, double x, double y){
    double r = sqrt(pow(x,2)+pow(y,2));
    if ((r<=0.5) || (r == 1) || (r>=1.5))
        return (0);
    else
        if ((r>0.5)&&(r<1))
            return (-A * exp(-pow(r-0.75,2)/(pow(0.25,2)-pow(r-0.75,2))));
        else
            if ((r > 1)&&(r<1.5))
                return (A * exp(-pow(r-1.25,2)/(pow(0.25,2)-pow(r-1.25,2))));
}
 
double SpeedController::uctr(double x, double y, double theta, double psi_t, double v){
    double A = 1.40;
    double r = sqrt(pow(x,2)+pow(y,2));
    return ((-var_phi(A,x,y)/r)*(x*cos(theta)+y*sin(theta)) + (var_phi(A,x,y)/r)*(-x*sin(theta)+y*cos(theta))*(1/(1+pow(v,2)))*psi_t/(1+pow(psi_t,2))) - tanh(v-0.5);
}
 
double SpeedController::tau(double x, double y, double theta, double psi_t, double v){
    double A = 1.40;
    double r = sqrt(pow(x,2)+pow(y,2));
    return ((-var_phi(A,x,y)/r)*(-x*sin(theta)+y*cos(theta))*(v/(1+pow(v,2)))*1/(1+pow(psi_t,2))-tanh(psi_t));
}
 
void SpeedController::Move_Han(double x0, double y0, double theta0, double w0, double v0){
    ros::Rate rate(1000);
    listener_.waitForTransform("/world", "/irobot1", ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform_;

    double dt =  0.016; // time step
    //initial conditions
    double var_theta0 = atan(y0/x0);
    double omega_t = (x0*sin(theta0)-y0*cos(theta0))*v0/(pow(x0,2)+pow(y0,2));
    
    double var_theta_t = var_theta0 + omega_t;
    double psi = theta0 - var_theta_t - M_PI/2;
    double u = uctr(x0,y0,theta0,psi,v0);
    double tt = tau(x0,y0,theta0,psi,v0);
    double w_t = w0 + tt;  
    double v_t = v0 + u; 
    //double omega_t = omega0 + tt;

    double var_theta_i1 = var_theta_t;
    double v_t_i1 = v_t;
    double w_t_i1 = w_t;
    //end of initial conditions


    ofstream fout;
    double t_start = ros::Time::now().toSec(); // starting time
    fout.open("test_12");
    fout << "v_ref: 0.39; ros rate: 100; ros sleep 0.1 second " <<"\n" ;

    while (nh_.ok()){
        try{
            double t_now = ros::Time::now().toSec(); // integrate function to this time 
            
            listener_.lookupTransform("/world", "/irobot1", ros::Time(0), transform_);//listen to current frame
            double current_roll,current_pitch,current_yaw; //get current yaw
            transform_.getBasis().getRPY(current_roll,current_pitch,current_yaw); 
            double current_x,current_y; //get current positions
            current_x = transform_.getOrigin().x();
            current_y = transform_.getOrigin().y();
            
            cout << "current v_t: " << v_t << " current angular: " << tt <<"\n";
            geometry_msgs::Twist vel_;  
            vel_.linear.x = v_t;
            vel_.angular.z = tt;
            cmd_vel_pub_.publish(vel_); //publish velocities
            
            fout << v_t << " " << tt << "\n";


            double last_w = w_t;
            double last_u = u;
            double last_tau = tt;
            double last_omega_t = omega_t;
            
            omega_t = omega(current_x,current_y,current_yaw,v_t);
            var_theta_t = var_theta_i1 + (omega_t + last_omega_t)*dt*0.5;
            
            psi = current_yaw - var_theta_t - M_PI/2;
            
            
            u = uctr(current_x,current_y,current_yaw,psi,v_t);
            tt = tau(current_x,current_y,current_yaw,psi,v_t);
 
            v_t = v_t_i1 + (u + last_u)*dt*0.5;
            cout << "new v_t: " << v_t << " new angular: " << tt <<"\n";
            w_t = w_t_i1 + (tt + last_tau)*dt*0.5;

            var_theta_i1 = var_theta_t;    
            v_t_i1 = v_t;
            w_t_i1 = w_t;
            
            double t_end = ros::Time::now().toSec();
            if ((t_end - t_start) >= 30)
                fout.close();
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.5).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}