#include "quimesis_segway_feedback_controller/segway_state_feedback_controller.h"
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


//#define GAZEBO

//#ifndef GAZEBO
  #include <quimesis_quimmotor/Command.h>
  #include <quimesis_quimmotor/Feedback.h>
  double MOTOR_REDUCTION = 49;
  double RESISTANCE = 2.5; //ohm
  double KV = 0.0327; // V/rpm = (1*60/(292*2*pi))
  double VMAX  = 24.0;
  double WHEEL_SEPARATION = 0.46;
  int TYPE_CONTROL = 0;
  double WHEEL_RADIUS = 0.0625;
  double Nticks =  980;
  double PI = 3.1415;
  double Nticks_ext = 8192;
  double I_max = 1.0;
   
//#endif



using namespace std;


double state_pitch, state_pitch_rate, state_linear_velocity,setpoint_speed,omega_ref,omega_mes,current,velLeft,velRight,velLeft_ext,velRight_ext;
 double command_ang,command_lin,vel_lin,vel_ang;
double count1 ;


void filterVelocityCallback(const geometry_msgs::Twist& msg){
   //Using the callback function just for subscribing
   //Subscribing the message and storing it in 'linx' and 'angZ'
   setpoint_speed  = msg.linear.x;
   omega_ref = msg.angular.z;

/*	if (setpoint_speed >-0.001 && setpoint_speed<0.001)
{

   if (setpoint_speed >= 0.0 ) {
        setpoint_speed = 0.0001;
    } else if (setpoint_speed < 0.0) {
        setpoint_speed = -0.0001;
    }



}*/


}

void callbackPitch(const std_msgs::Float64::ConstPtr& msg) {
    state_pitch = msg->data;
}

void callbackPitchRate(const std_msgs::Float64::ConstPtr& msg) {
    state_pitch_rate = msg->data;
}

 /*#ifdef GAZEBO





void callbackVel(const std_msgs::Float64::ConstPtr& msg) {
    state_linear_velocity = msg->data;
   }

void callbackAng(const std_msgs::Float64::ConstPtr& msg) {
    omega_mes = msg->data;
   }


 #else*/
double voltage_to_current(double voltage, double vel_wheel) {


return (voltage * VMAX - KV* MOTOR_REDUCTION * ((vel_wheel / WHEEL_RADIUS) - state_pitch_rate )) / RESISTANCE;

}



void leftFeedbackCallback(const quimesis_quimmotor::Feedback::ConstPtr& cmd)
{

    velLeft = - ((cmd->measured_velocity)*WHEEL_RADIUS*2*PI)/Nticks; //meters per seconds

   
    velLeft_ext = - ((cmd->measured_velocity_ext)*WHEEL_RADIUS*2*PI)/Nticks_ext; //meters per seconds


   // ROS_INFO("Velocity left: lin = %f ", velLeft);

}

void rightFeedbackCallback(const quimesis_quimmotor::Feedback::ConstPtr& cmd)
{

    velRight = - ((cmd->measured_velocity)*WHEEL_RADIUS*2*PI)/Nticks; //meters per seconds

    velRight_ext = - ((cmd->measured_velocity_ext)*WHEEL_RADIUS*2*PI)/Nticks_ext; //meters per seconds


   // ROS_INFO("Velocity right: lin = %f ", velRight);

}




 //#endif


int main(int argc, char** argv) {
    ros::init(argc, argv, "segway_state_feedback_controller");

    ros::NodeHandle nh;

    SegwayStateFeedbackController controller_speed;
    SegwayStateFeedbackController controller_angular;




    // Get parameters
    std::vector<double> a, b, l, k;
    double ki;
    double Windupmax;
    double ramp_lin,ramp_ang,SatMax;
    double KP_ang,KI_ang;


    if (ros::param::get("K", k)) {
        controller_speed.setGain(k);
    } else {ROS_WARN("Parameter K not found");
    }



    if (ros::param::get("KI", ki)) {
        controller_speed.setintegralCoef(ki);
	controller_angular.setintegralCoef(ki);

    } else {
        ROS_WARN("Parameter Ki not found");
    }

    if (ros::param::get("WMAX",Windupmax )) {
        controller_speed.setwindupCoef(Windupmax);
	controller_angular.setwindupCoef(Windupmax);

    } else {
        ROS_WARN("Parameter WMAX not found");
    }

/*if (ros::param::get("SAT",SatMax )) {
        controller_speed.setsatCoef(SatMax);

    } else {
        ROS_WARN("Parameter SAT not found");
    }*/


	if (ros::param::get("RAMP_lin", ramp_lin)) {
        controller_speed.setRampCoef(ramp_lin);
    } else {
        ROS_WARN("Parameter Ramp_lin not found");
    }


	if (ros::param::get("RAMP_ang", ramp_ang)) {
	controller_angular.setRampCoef(ramp_ang);
    } else {
        ROS_WARN("Parameter Ramp_ang not found");
    }


	if (ros::param::get("ki", KI_ang)) {
	controller_angular.setI(KI_ang);
    } else {
        ROS_WARN("Parameter ki not found");
    }

   if (ros::param::get("kp", KP_ang)) {
	controller_angular.setP(KP_ang);
    } else {
        ROS_WARN("Parameter ko not found");
    }


    // Subscribe to speed data + topics IMU
   

    ros::Subscriber sub = nh.subscribe("/cmd_vel",1,&filterVelocityCallback);
    ros::Subscriber pitch_sub = nh.subscribe("segway/state/pitch", 1, callbackPitch);
    ros::Subscriber pitchrate = nh.subscribe("segway/state/pitch_rate", 1, callbackPitchRate);
    ros::Publisher setpoint_lin_pub = nh.advertise<std_msgs::Float64>("linear/setpoint", 1);
    ros::Publisher setpoint_ang_pub = nh.advertise<std_msgs::Float64>("angular/setpoint", 1);

    //#ifdef GAZEBO

    

    //ros::Subscriber vel_sub = nh.subscribe("/gazebo/wheel_speed", 1, callbackVel);
    //ros::Subscriber ang_sub = nh.subscribe("/gazebo/angular_vel", 1, callbackAng);


    // Publisher to publish the effort command from the controller
    ros::Publisher effort_pub = nh.advertise<std_msgs::Float64>("segway/controller/cmd", 1);
    ros::Publisher effort_pub1 = nh.advertise<std_msgs::Float64>("segway/controller/current_cmd", 1); 
    ros::Publisher effort_pub2 = nh.advertise<std_msgs::Float64>("segway/controller/pwm", 1);
    ros::Publisher effort_pub_ang = nh.advertise<std_msgs::Float64>("segway/controller/cmd_ang", 1);
 
    
     ros::Publisher target_speed_pub3 = nh.advertise<std_msgs::Float64>("segway/time", 1);


    //#else

  

    ros::Subscriber leftFeedback = nh.subscribe("/left/feedback", 1, leftFeedbackCallback);
    ros::Subscriber rightFeedback = nh.subscribe("/right/feedback", 1, rightFeedbackCallback);

    ros::Publisher left_pub=nh.advertise<quimesis_quimmotor::Command>("/left/cmd", 1);
    ros::Publisher right_pub=nh.advertise<quimesis_quimmotor::Command>("/right/cmd", 1);
  //#endif



    // Perform the control loop at 50Hz
    ros::Rate rate(50);

    while(ros::ok()) {

        std_msgs::Float64 cmd_lin,cmd_ang, omega_r,speed_r ,cmd_lin_current , pwm_command,time;
       

        //#ifdef GAZEBO
	//controller_speed.update_speed(setpoint_speed,state_linear_velocity,state_pitch,state_pitch_rate);
        //controller_angular.update_omega(omega_ref,omega_mes);
        //#else
        time.data = controller_speed.get_time();
        //vel_lin = (velRight + velLeft) /2;
        //vel_ang = (velLeft - velRight)/(WHEEL_SEPARATION);

         vel_lin = (velRight_ext + velLeft_ext) /2;
         vel_ang = (velLeft_ext - velRight_ext)/(WHEEL_SEPARATION);
        controller_speed.update_speed(setpoint_speed,vel_lin,state_pitch,state_pitch_rate);
        controller_angular.update_omega(omega_ref,vel_ang);
        //#endif

        cmd_lin.data = controller_speed.getEffortCmd();
        cmd_ang.data = controller_angular.getEffort_Ang_Cmd();
	omega_r.data = omega_ref;
	speed_r.data = setpoint_speed;
        command_ang = controller_angular.getEffort_Ang_Cmd();
        command_lin = controller_speed.getEffortCmd();
        
        if (command_lin >= I_max)
       {
       command_lin = I_max;  
       }

       else if (command_lin <= -I_max)
       { 
      command_lin = -I_max; 
       }

        //Declares the message to be sent
        //#ifdef GAZEBO
        effort_pub.publish(cmd_lin);
        effort_pub_ang.publish(cmd_ang);
	

         //#else

        setpoint_lin_pub.publish(speed_r);
	setpoint_ang_pub.publish(omega_r);
        target_speed_pub3.publish(time);

      quimesis_quimmotor::Command left, right;

      if  (TYPE_CONTROL == 0)  // For current control
{     
      left.loop_function = 1;
       
/*  if (count1 < 250)
  
{
left.setpoint = 100.0 ;
right.setpoint = 100.0 ;
}

else if ((count1 >= 250) &&(count1 < 500)) 
{
left.setpoint = 200.0;
right.setpoint = 200.0 ;
}

else if ((count1 >= 500) &&(count1 < 750)) 
{
left.setpoint = 300.0 ;
right.setpoint = 300.0 ;
}


else 
{
right.setpoint = 0;
left.setpoint = 0;
}

*/


      
      left.setpoint = -1000*(command_lin+command_ang);
      left.request_id = 5;
   
      cmd_lin_current.data =1000*(command_lin+command_ang);
      effort_pub1.publish(cmd_lin_current);
      

   
      
      right.loop_function = 1;
      right.setpoint = -1000*(command_lin-command_ang);
      right.request_id = 5;
      
}

      else                     // For voltage control
{      
      double pwm_cmd = (command_lin)*(128/VMAX); 
      
      left.loop_function = 0;
      left.setpoint = pwm_cmd;  // Transform voltage in pwm
      left.request_id = 5;
    
      pwm_command.data = pwm_cmd;
      effort_pub2.publish(pwm_command);

      right.loop_function = 0;
      right.setpoint = pwm_cmd;  // Transform voltage in pwm
      right.request_id = 5;
}


   

      left_pub.publish(left);
      right_pub.publish(right);

        //#endif


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

