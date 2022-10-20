#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include "math.h"
#include "ros/ros.h"
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include "nav_msgs/Path.h"
 #include <quimesis_quimmotor/Command.h>
  #include <quimesis_quimmotor/Feedback.h>

using namespace std;
  double MOTOR_REDUCTION = 49;
  double RESISTANCE = 1.68;
  double EMF = 0.0327;
  double VMAX  = 24.0;
  double WHEEL_SEPARATION = 0.277;
  int TYPE_CONTROL = 0;
  double WHEEL_RADIUS = 0.0625;
  double Nticks =  980;
  double PI = 3.1415;

double setpoint_speed , omega_ref;
double state_pitch, state_pitch_rate,velLeft,current_left,voltage_left,pwm_left,velRight,velRight_ext,velLeft_ext,current_right,voltage_right,pwm_right,current_right_cmd,current_left_cmd,vel_setpoint,ang_setpoint;




void callbackPitch(const std_msgs::Float64::ConstPtr& msg) {
    state_pitch = msg->data;
}



void callbackPitchRate(const std_msgs::Float64::ConstPtr& msg) {
    state_pitch_rate = msg->data;
}




void callbacksetpoint_vel(const std_msgs::Float64::ConstPtr& msg) {
    vel_setpoint = msg->data;
}

void callbacksetpoint_ang(const std_msgs::Float64::ConstPtr& msg) {
    ang_setpoint = msg->data;
}



 void leftFeedbackCallback(const quimesis_quimmotor::Feedback::ConstPtr& cmd)
{

    velLeft = (cmd->measured_velocity);//*WHEEL_RADIUS*2*PI/MOTOR_REDUCTION;
    velLeft_ext = (cmd->measured_velocity_ext);//*WHEEL_RADIUS*2*PI/MOTOR_REDUCTION;
    current_left = cmd->measured_current;
   


   // ROS_INFO("Velocity left: lin = %f ", velLeft);

}

void rightFeedbackCallback(const quimesis_quimmotor::Feedback::ConstPtr& cmd)
{

    velRight = (cmd->measured_velocity);//*WHEEL_RADIUS*2*PI/MOTOR_REDUCTION;
    velRight_ext = (cmd->measured_velocity_ext);//*WHEEL_RADIUS*2*PI/MOTOR_REDUCTION;
    current_right = cmd->measured_current;
  


   // ROS_INFO("Velocity left: lin = %f ", velLeft);

}

void leftCommandCallback(const quimesis_quimmotor::Command::ConstPtr& cmd)
{

    current_right_cmd = cmd->setpoint;
    current_left_cmd = cmd->setpoint;
 }



int main(int argc, char** argv){
  ros::init(argc, argv, "printdata");

  ros::NodeHandle nh;
  
   
     // Subscribe to IMU data
    ros::Subscriber pitch_sub = nh.subscribe("segway/state/pitch", 1, callbackPitch);
    ros::Subscriber pitchrate = nh.subscribe("segway/state/pitch_rate", 1, callbackPitchRate);

    

 
    ros::Subscriber leftFeedback = nh.subscribe("/left/feedback", 1, leftFeedbackCallback);
   
    
    ros::Subscriber leftCommand = nh.subscribe("/left/cmd", 1, leftCommandCallback);

    ros::Subscriber rightFeedback = nh.subscribe("/right/feedback", 1, rightFeedbackCallback);


  
    ros::Subscriber velFeedback = nh.subscribe("linear/setpoint", 1, callbacksetpoint_vel);

ros::Subscriber angFeedback = nh.subscribe("angular/setpoint", 1, callbacksetpoint_ang);
  


  ofstream myfile;
myfile.open ("/opt/ros/melodic.txt");



     

  ros::Rate rate(50.0);
  while(nh.ok()){

    

    myfile << state_pitch << " " << state_pitch_rate << " " << velLeft << " " <<velLeft_ext << " " << current_left << " " << current_left_cmd << " " << velRight << " " << velRight_ext << " " << current_right << " " << current_right_cmd <<  " " << vel_setpoint << " " << ang_setpoint << endl;           
  
    
    ros::spinOnce();
    rate.sleep();
  }


myfile.close();
}
