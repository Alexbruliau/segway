#ifndef SEGWAY_FEEDBACK_CONTROLLER_H
#define  SEGWAY_FEEDBACK_CONTROLLER_H

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <ros/time.h>
#include <iostream>

#define N 2
struct IMUdata {

    struct Orientation {
        Orientation() : x(0), y(0), z(0), w(0) {}
        double x;
        double y;
        double z;
        double w;
    } orientation;
    struct AngularVelocity {
        AngularVelocity() : x(0), y(0), z(0) {}
        double x;
        double y;
        double z;
    } angular_velocity;
    struct LinearAcceleration {
        LinearAcceleration() : x(0), y(0), z(0) {}
        double x;
        double y;
        double z;
    } linear_acceleration;
};

class SegwayStateFeedbackController
{
  private:
    double command;
    double command_ang;

    double error;
    double integral_error;

    double error_ang;
    double integral_error_ang;

    ros::Time prev_time;
    ros::Duration delta_t;

    ros::Time prev_time1;
    ros::Duration delta_t1;


    double Imax;




    double Ramp;
    double gain[2];
    double Kp_ang,Ki_ang;

    
    
    
    double K11[2][2];
    double Ki;
    double Ts;
    double Kint;
    int loop_counter;
    int loop_counter1;
    int ramp_limit;
    double target_speed;
    double target_ang;
    double ts ;
    double speed_mesured;
    double speed_ang_mes;
    //double x_est1,x_est2;
     //double x_est1_new,x_est2_new;


    bool windup;
    double windupMax;

    IMUdata imu_data;



  public:
    SegwayStateFeedbackController();
    ~SegwayStateFeedbackController();


    void update_speed(double speed, double speed_mes,double state_pitch,double state_pitch_rate);
    void update_omega(double omega_ref,double omega_mes);
    void update_estimate(double pitch,double pitch_rate, double u,double x_est1,double x_est2 , double x_est3) ;
    void multiplyMatrices3(double firstMatrix[3][3], double secondMatrix[3][3], double mult[3][3]);

    void multiplyMatrices23(double firstMatrix[2][3], double secondMatrix[3][3], double mult[2][3]);

    void multiplyMatrices32(double firstMatrix[2][3], double secondMatrix[3][2], double mult[2][2]);

    void multiplyMatrices23bis(double firstMatrix[3][3], double secondMatrix[3][2], double mult[3][2]);

    void multiplyMatrices32bis(double firstMatrix[3][2], double secondMatrix[2][2], double mult[3][2]);

    void multiplyMatrices323(double firstMatrix[3][2], double secondMatrix[2][3], double mult[3][3]);
  
    void transpose(double fM[2][2], double tr[2][2]);
    void transposeMatrices_inv(double firstMatrix[2][2], double trans[2][2]);
    double DET(double M[2][2], int n);

    bool INV(double M[2][2], double inv[2][2]);
   double determinant(double A[N][N], int n) ;
  
   void adjoint(double A[N][N],double adj[N][N]) ;
    
    void getCofactor(double M[N][N], double t[N][N], int p, int q, int n);


   bool inverse(double A[N][N], double inverse[N][N]) ;

    void addMatrices2(double firstMatrix[2][2], double secondMatrix[2][2], double su[2][2]);


    void addMatrices3(double firstMatrix[3][3], double secondMatrix[3][3], double su[3][3]);
 
    void subMatrices(double firstMatrix[3][3], double secondMatrix[3][3], double diff[3][3]);
    
    double getEffortCmd();
    double getEffort_Ang_Cmd();
    double getEstimate();
    double get_error();
    double get_integral_error();
    double get_integral_error_ang();
    double get_target_speed();
    double get_pitch();
    double get_pitch_rate();
    double get_vel();
    double get_time();
    void print_state_linear();
    void print_state_angular();
    int sgn(double v);

    double antiWindup(double I);

    void setA(std::vector<double> a);
    void setB(std::vector<double> b);
    void setL(std::vector<double> c);
    void setGain(std::vector<double> k);
    void setPrecompCoef(double n);
    void setintegralCoef(double ki);
    void setP(double KP) ;
    void setI(double KI) ;
    void setTsampling(double ts);
    void setwindupCoef(double Windupmax);
    void setRampCoef (double ramp);

};

#endif



