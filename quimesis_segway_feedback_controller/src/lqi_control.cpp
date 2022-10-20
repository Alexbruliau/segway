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
using namespace std;

#define N 2


double A1[3][3];
double B1[3];
double I[3][3];
double Q1[2][2];
double Q2[3][3];
double Ht[3][2];
double At[3][3];
double KL[3][2];
double P[3][3];
double H[2][3];
double K1[3][2];
double K2[2][2];
double K3[2][2];
double K4[2][2];
double K5[3][3];
double K6[3][3];
double K12[3][3];
double D[3][3];

double x_estpitch;
double x_estrate;
double x_estvel;
/*double A1[3][3];
double B1[3];
double I[3][3];
double Q1[3][3];
double Q2[3][3];
double Ht[3][3];
double At[3][3];
double KL[3][3];
double P[3][3];
double H[3][3];
double K1[3][3];
double K2[3][3];
double K3[3][3];
double K4[3][3];
double K5[3][3];
double K6[3][3];
double K12[3][3];
double D[3][3];*/
//double Kp = -0.04;




SegwayStateFeedbackController::SegwayStateFeedbackController() {
    command = 0;
    command_ang = 0;


    error = 0;
    error_ang = 0;
    Kint = 0;

    integral_error = 0;
    integral_error_ang = 0;
    ramp_limit = 0.1;
    target_speed = 0;
    target_ang = 0;
    ts = 0.02;
    speed_mesured = 0.0;
    speed_ang_mes = 0.0;



    windup = true;
    prev_time = ros::Time::now();
    delta_t = ros::Time::now() - ros::Time::now();

    prev_time1 = ros::Time::now();
    delta_t1 = ros::Time::now() - ros::Time::now();


    imu_data.orientation.x = 0;
    imu_data.orientation.y = 0;
    imu_data.orientation.z = 0;
    imu_data.orientation.w = 1;

    imu_data.angular_velocity.x = 0;
    imu_data.angular_velocity.y = 0;
    imu_data.angular_velocity.z = 0;

    imu_data.linear_acceleration.x = 0;
    imu_data.linear_acceleration.y = 0;
    imu_data.linear_acceleration.z = 0;

    loop_counter = 0;
    loop_counter1 = 0;
  
    

    


}

SegwayStateFeedbackController::~SegwayStateFeedbackController() {}

//Kalman filter

void SegwayStateFeedbackController::update_estimate(double pitch,double pitch_rate, double u,double x_est1,double x_est2 , double x_est3) 
{





A1[0][0] = 1.0023,A1[0][1] = 0.02,A1[1][0] = 0.2301,A1[1][1] = 1.002,A1[0][2] = 0.0000 ,A1[1][2] = 0.0042 , A1[2][2] = 0.9969 ,A1[2][0] = -0.0335 ,A1[2][1] = -0.0001   ;
B1[0] =-0.002,B1[1] = -0.1962,B1[2] = 0.1456;
I[0][0] = 1.0,I[0][1] = 0.0,I[1][0] = 0.0,I[1][1] = 1.0 ,I[0][2] = 0.000 ,I[1][2] = 0.0 , I[2][2] = 1.0 ,I[2][0] = 0.0 ,I[2][1] = 0.0   ;
Q2[0][0] = 0.0000,Q2[0][1] = 0.0,Q2[1][0] = 0.0,Q2[1][1] = 0.001 ,Q2[0][2] = 0.000 ,Q2[1][2] = 0.0 , Q2[2][2] = 0.0,Q2[2][0] = 0.0 ,Q2[2][1] = 0.0;
P[0][0] = 0.000001,P[1][0] = 0.0,P[0][1] = 0.0,P[1][1] = 0.00009,P[0][2] = 0.000 ,P[1][2] = 0.0 , P[2][2] = 0.1 ,P[2][0] = 0.0 ,P[2][1] = 0.0  ;
KL[0][0] = 0.0 ,KL[0][1] = 0.0,KL[1][0] = 0.0,KL[1][1] = 0.0,KL[2][0] = 0.0 ,KL[2][1] = 0.0;
H[0][0] = 1.0,H[1][0] = 0.0,H[0][1] = 0.0,H[1][1] = 1.0 , H[1][2] = 0.0,H[0][2] = 0.0;
Ht[0][0] = 1.0,Ht[1][0] = 0.0,Ht[0][1] = 0.0,Ht[1][1] = 1.0 , Ht[2][1] = 0.0,Ht[2][0] = 0.0;
At[0][0] = A1[0][0],At[1][0] =A1[0][1],At[0][1] = A1[1][0],At[1][1] = A1[1][1], At[2][0] = A1[0][2] ,At[2][1] = A1[1][2] , At[2][2] = A1[2][2] ,At[0][2] = A1[2][0] ,At[1][2] = A1[2][1];
K1[0][0] = 0.0,K1[1][0] = 0.0,K1[0][1] = 0.0,K1[1][1] = 0.0 ,K1[2][0] = 0.0,K1[2][1] = 0.0;
K2[0][0] = 0.0,K2[1][0] = 0.0,K2[0][1] = 0.0,K2[1][1] = 0.0;
K3[0][0] = 0.0,K3[1][0] = 0.0,K3[0][1] = 0.0,K3[1][1] = 0.0;
K4[0][0] = 0.0,K4[1][0] = 0.0,K4[0][1] = 0.0,K4[1][1] = 0.0;
K5[0][0] = 0.0,K5[1][0] = 0.0,K5[0][1] = 0.0,K5[1][1] = 0.0 , K5[0][2] = 0.000 ,K5[1][2] = 0.0 , K5[2][2] = 0.0 ,K5[2][0] = 0.0 ,K5[2][1] = 0.0   ;

K6[0][0] = 0.0,K6[1][0] = 0.0,K6[0][1] = 0.0,K6[1][1] = 0.0 , K6[0][2] = 0.000 ,K6[1][2] = 0.0 , K6[2][2] = 0.0 ,K6[2][0] = 0.0 ,K6[2][1] = 0.0   ;
K12[0][0] = 0.0,K12[1][0] = 0.0,K12[0][1] = 0.0,K12[1][1] = 0.0 ,K12[0][2] = 0.000 ,K12[1][2] = 0.0 , K12[2][2] = 0.0 ,K12[2][0] = 0.0 ,K12[2][1] = 0.0 ;
D[0][0] = 0.0,D[1][0] = 0.0,D[0][1] = 0.0,D[1][1] = 0.0 , D[0][2] = 0.000 ,D[1][2] = 0.0 , D[2][2] = 0.0 ,D[2][0] = 0.0 ,D[2][1] = 0.0   ;
Q1[0][0] = 0.000000212,Q1[0][1] = 0.0,Q1[1][0] = 0.0,Q1[1][1] = 0.00003364;




// Predict Update

x_est1 = A1[0][0]*x_est1 + A1[0][1]*x_est2 + A1[0][2]*x_est3 + B1[0]*u;

x_est2 = A1[1][0]*x_est1+ A1[1][1]*x_est2 +  A1[1][2]*x_est3 + B1[1]*u;

//x_est3 = A1[2][0]*x_est1+ A1[2][1]*x_est2 +  A1[2][2]*x_est3 + B1[2]*u;

multiplyMatrices3(A1,P,D);


multiplyMatrices3(D,At,K12);



addMatrices3(K12,Q2,P);
//ROS_INFO("pitch :%f,pitchrate :%f,x_est1 :%f,x_est2 :%f",pitch,pitch_rate,x_est1,x_est2);
// Measurement Update



multiplyMatrices23bis(P,Ht,K1);
multiplyMatrices32(H,K1,K2);

addMatrices2(K2,Q1,K3);
transposeMatrices_inv(K3,K4);
multiplyMatrices32bis(K1,K4,KL);


x_est1 +=  KL[0][0]*(pitch - x_est1) + KL[0][1]*(pitch_rate - x_est2) ;
 
x_est2 +=  KL[1][1]*(pitch_rate - x_est2) + KL[1][0]*(pitch - x_est1) ;

//x_est3 +=  KL[2][1]*(pitch_rate - x_est2) + KL[2][0]*(pitch - x_est1) ;



ROS_INFO("pitch :%f,pitchrate :%f,x_est1 :%f,x_est2 :%f,x_est3 :%f,x_est4 :%f",KL[0][0],KL[0][1],KL[1][0],KL[1][1],KL[2][1],KL[2][0]);

x_estpitch = x_est1;
x_estrate = x_est2;
x_estvel = x_est3;
//Covariance Update

multiplyMatrices323(KL,H,K5);
subMatrices(I,K5,K6);
multiplyMatrices3(K6,P,P);




}




void SegwayStateFeedbackController::multiplyMatrices3(double firstMatrix[3][3], double secondMatrix[3][3], double mult[3][3])
{
	int i, j, k;



// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			mult[i][j] = 0.0;
		}
	}
	

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			for(k = 0; k <= 2; ++k)
			{
				mult[i][j] = mult[i][j] + firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}


}


void SegwayStateFeedbackController::multiplyMatrices23(double firstMatrix[2][3], double secondMatrix[3][3], double mult[2][3])
{
	int i, j, k;



// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 1; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			mult[i][j] = 0.0;
		}
	}
	

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i <= 1; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			for(k = 0; k <= 2; ++k)
			{
				mult[i][j] = mult[i][j] + firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}


}

void SegwayStateFeedbackController::multiplyMatrices32(double firstMatrix[2][3], double secondMatrix[3][2], double mult[2][2])
{
	int i, j, k;



// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 1; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			mult[i][j] = 0.0;
		}
	}
	

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i <= 1; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			for(k = 0; k <= 2; ++k)
			{
				mult[i][j] = mult[i][j] + firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}


}

void SegwayStateFeedbackController::multiplyMatrices23bis(double firstMatrix[3][3], double secondMatrix[3][2], double mult[3][2])
{
	int i, j, k;



// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			mult[i][j] = 0.0;
		}
	}
	

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			for(k = 0; k <= 2; ++k)
			{
				mult[i][j] = mult[i][j] + firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

void SegwayStateFeedbackController::multiplyMatrices32bis(double firstMatrix[3][2], double secondMatrix[2][2], double mult[3][2])
{
	int i, j, k;



// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			mult[i][j] = 0.0;
		}
	}
	

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			for(k = 0; k <= 1; ++k)
			{
				mult[i][j] = mult[i][j] + firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

void SegwayStateFeedbackController::multiplyMatrices323(double firstMatrix[3][2], double secondMatrix[2][3], double mult[3][3])
{
	int i, j, k;



// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			mult[i][j] = 0.0;
		}
	}
	

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			for(k = 0; k <= 1; ++k)
			{
				mult[i][j] = mult[i][j] + firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

void SegwayStateFeedbackController::addMatrices3(double firstMatrix[3][3], double secondMatrix[3][3], double su[3][3])
{
	int i, j;

	// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			su[i][j] = 0.0;
		}
	}

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	
       for (i = 0;i<=2; ++i ) { 
     for (j = 0;j<=2; ++j ) {
       su[i][j]=firstMatrix[i][j]+secondMatrix[i][j];
     }
   }

}


void SegwayStateFeedbackController::addMatrices2(double firstMatrix[2][2], double secondMatrix[2][2], double su[2][2])
{
	int i, j;

	// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 1; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			su[i][j] = 0.0;
		}
	}

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	
       for (i = 0;i<=1; ++i ) { 
     for (j = 0;j<=1; ++j ) {
       su[i][j]=firstMatrix[i][j]+secondMatrix[i][j];
     }
   }

}



void SegwayStateFeedbackController::transpose(double fM[2][2], double tr[2][2])
{
	int i, j;

	// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 1; ++i)
	{
		for(j = 0; j <= 1; ++j)
		{
			tr[i][j] = 0.0;
		}
	}

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	
 tr[0][0] = fM[0][0];
tr[1][0] =fM[0][1];
tr[0][1] = fM[1][0];
tr[1][1] = fM[1][1];
   

}


void SegwayStateFeedbackController::subMatrices(double firstMatrix[3][3], double secondMatrix[3][3], double diff[3][3])
{
	int i, j, k;

	// Initializing elements of matrix mult to 0.
	for(i = 0; i <= 2; ++i)
	{
		for(j = 0; j <= 2; ++j)
		{
			diff[i][j] = 0.0;
		}
	}

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	
       for (i = 0;i<=2;i++ ) { 
     for (j = 0;j<=2;j++ ) {
       diff[i][j]=firstMatrix[i][j]-secondMatrix[i][j];
     }
   }

}




void SegwayStateFeedbackController::transposeMatrices_inv(double firstMatrix[2][2], double trans[2][2])
{
	int i, j, k,l;

	for(i = 0; i <= 1; ++i) 
{    
    for(j = 0; j <= 1; ++j)
		{
			trans[i][j] = 0.0;
		}
}

	double det1 = DET(firstMatrix, 2);

            trans[0][0]=firstMatrix[1][1]/det1;
            trans[1][1]=firstMatrix[0][0]/det1;
            trans[0][1]= -firstMatrix[0][1]/det1;
            trans[1][0]= -firstMatrix[1][0]/det1;
  
}


double SegwayStateFeedbackController::DET(double M[N][N], int n) //to find determinant
 {
    double det = 0.0;
   double submatrix[N][N];
   if (n == 2)
{
      return ((M[0][0] * M[1][1]) - (M[0][1] * M[1][0]));
}

}




// Function to get cofactor of A[p][q] in temp[][]. n is current 
// dimension of A[][] 


/*void SegwayStateFeedbackController::update_estimate(double pitch,double pitch_rate, double speed ,double u,double x_est1,double x_est2 , double x_est3) 
{




A1[0][0] = 1.0023,A1[0][1] = 0.02,A1[1][0] = 0.2301,A1[1][1] = 1.002,A1[0][2] = 0.0000 ,A1[1][2] = 0.0042 , A1[2][2] = 0.9969 ,A1[2][0] = -0.0335 ,A1[2][1] = -0.0001   ;
B1[0] =-0.002,B1[1] = -0.1962,B1[2] = 0.1456;
I[0][0] = 1.0,I[0][1] = 0.0,I[1][0] = 0.0,I[1][1] = 1.0 ,I[0][2] = 0.000 ,I[1][2] = 0.0 , I[2][2] = 1.0 ,I[2][0] = 0.0 ,I[2][1] = 0.0   ;
Q2[0][0] = 0.0,Q2[0][1] = 0.0,Q2[1][0] = 0.0,Q2[1][1] = 0.0 ,Q2[0][2] = 0.000 ,Q2[1][2] = 0.0 , Q2[2][2] = 0.0,Q2[2][0] = 0.0 ,Q2[2][1] = 0.0;
P[0][0] = 0.000001,P[1][0] = 0.0,P[0][1] = 0.0,P[1][1] = 0.0001,P[0][2] = 0.000 ,P[1][2] = 0.0 , P[2][2] = 0.000001 ,P[2][0] = 0.0 ,P[2][1] = 0.0  ;
KL[0][0] = 0.0 ,KL[0][1] = 0.0,KL[1][0] = 0.0,KL[1][1] = 0.0,KL[0][2] = 0.0 ,KL[1][2] = 0.0 , KL[2][2] = 0.0 ,KL[2][0] = 0.0 ,KL[2][1] = 0.0;
H[0][0] = 1.0,H[1][0] = 0.0,H[0][1] = 0.0,H[1][1] = 1.0 , H[0][2] = 0.0 ,H[1][2] = 0.0 , H[2][2] = 1.0 ,H[2][0] = 0.0 ,H[2][1] = 0.0;
Ht[0][0] = 1.0,Ht[1][0] = 0.0,Ht[0][1] = 0.0,Ht[1][1] = 1.0 ,Ht[0][2] = 0.0 ,Ht[1][2] = 0.0 , Ht[2][2] = 1.0 ,Ht[2][0] = 0.0 ,Ht[2][1] = 0.0;
At[0][0] = A1[0][0],At[1][0] =A1[0][1],At[0][1] = A1[1][0],At[1][1] = A1[1][1], At[2][0] = A1[0][2] ,At[2][1] = A1[1][2] , At[2][2] = A1[2][2] ,At[0][2] = A1[2][0] ,At[1][2] = A1[2][1];
K1[0][0] = 0.0,K1[1][0] = 0.0,K1[0][1] = 0.0,K1[1][1] = 0.0 ,K1[0][2] = 0.0 ,K1[1][2] = 0.0 , K1[2][2] = 0.0 ,K1[2][0] = 0.0 ,K1[2][1] = 0.0;
K2[0][0] = 0.0,K2[1][0] = 0.0,K2[0][1] = 0.0,K2[1][1] = 0.0 , K2[0][2] = 0.0 ,K2[1][2] = 0.0 , K2[2][2] = 0.0 ,K2[2][0] = 0.0 ,K2[2][1] = 0.0;
K3[0][0] = 0.0,K3[1][0] =0.0,K3[0][1] = 0.0,K3[1][1] = 0.0 , K3[0][2] = 0.0 ,K3[1][2] = 0.0 , K3[2][2] = 0.0 ,K3[2][0] = 0.0 ,K3[2][1] = 0.0;
K4[0][0] = 0.0,K4[1][0] = 0.0,K4[0][1] = 0.0,K4[1][1] = 0.0 ,K4[0][2] = 0.0 ,K4[1][2] = 0.0 , K4[2][2] = 0.0 ,K4[2][0] = 0.0 ,K4[2][1] = 0.0;  
K5[0][0] = 0.0,K5[1][0] = 0.0,K5[0][1] = 0.0,K5[1][1] = 0.0 , K5[0][2] = 0.000 ,K5[1][2] = 0.0 , K5[2][2] = 0.0 ,K5[2][0] = 0.0 ,K5[2][1] = 0.0   ;

K6[0][0] = 0.0,K6[1][0] = 0.0,K6[0][1] = 0.0,K6[1][1] = 0.0 , K6[0][2] = 0.000 ,K6[1][2] = 0.0 , K6[2][2] = 0.0 ,K6[2][0] = 0.0 ,K6[2][1] = 0.0   ;
K12[0][0] = 0.0,K12[1][0] = 0.0,K12[0][1] = 0.0,K12[1][1] = 0.0 ,K12[0][2] = 0.000 ,K12[1][2] = 0.0 , K12[2][2] = 0.0 ,K12[2][0] = 0.0 ,K12[2][1] = 0.0 ;
D[0][0] = 0.0,D[1][0] = 0.0,D[0][1] = 0.0,D[1][1] = 0.0 , D[0][2] = 0.000 ,D[1][2] = 0.0 , D[2][2] = 0.0 ,D[2][0] = 0.0 ,D[2][1] = 0.0   ;
Q1[0][0] = 0.000000212,Q1[0][1] = 0.0,Q1[1][0] = 0.0,Q1[1][1] = 0.00003364 , Q1[0][2] = 0.0 ,Q1[1][2] = 0.0 , Q1[2][2] = 0.0 ,Q1[2][0] = 0.0 ,Q1[2][1] = 0.0;




// Predict Update

x_est1 = A1[0][0]*x_est1 + A1[0][1]*x_est2 + A1[0][2]*x_est3 + B1[0]*u;

x_est2 = A1[1][0]*x_est1+ A1[1][1]*x_est2 +  A1[1][2]*x_est3 + B1[1]*u;

x_est3 = A1[2][0]*x_est1+ A1[2][1]*x_est2 +  A1[2][2]*x_est3 + B1[2]*u;

multiplyMatrices3(A1,P,D);


multiplyMatrices3(D,At,K12);



addMatrices3(K12,Q2,P);
//ROS_INFO("pitch :%f,pitchrate :%f,x_est1 :%f,x_est2 :%f",pitch,pitch_rate,x_est1,x_est2);
// Measurement Update



multiplyMatrices3(P,Ht,K1);
multiplyMatrices3(H,K1,K2);

addMatrices3(K2,Q1,K3);
inverse(K3,K4);

multiplyMatrices3(K1,K4,KL);


x_est1 +=  KL[0][0]*(pitch - x_est1) + KL[0][1]*(pitch_rate - x_est2) + KL[0][2]*(speed - x_est3);
 
x_est2 +=  KL[1][1]*(pitch_rate - x_est2) + KL[1][0]*(pitch - x_est1)  + KL[1][2]*(speed - x_est3);

x_est3 +=  KL[2][1]*(pitch_rate - x_est2) + KL[2][0]*(pitch - x_est1)  + KL[2][2]*(speed - x_est3);

/*x_est1 = x_est1 +  0.1177*(pitch - x_est1) + 0.0025*(pitch_rate - x_est2) + 0*(speed - x_est3) ;
 
x_est2 = x_est2 + 0.0085*(pitch_rate - x_est2) + 0.3966*(pitch - x_est1)  + 0*(speed - x_est3);

x_est3 = x_est3 -0.0012*(pitch_rate - x_est2) -0.0548*(pitch - x_est1)  + 0*(speed - x_est3);

//ROS_INFO("pitch :%f",deto);


ROS_INFO("a1 :%f,a2 :%f,a3 :%f,a4 :%f,a5 :%f,a6 :%f,a7 :%f,a8 :%f,a9 :%f",KL[0][0],KL[0][1],KL[0][2],KL[1][0],KL[1][1],KL[1][2],KL[2][0],KL[2][1],KL[2][2]);

x_estpitch = x_est1;
x_estrate = x_est2 ;
x_estvel = x_est3;
//Covariance Update

multiplyMatrices3(KL,H,K5);
subMatrices(I,K5,K6);
multiplyMatrices3(K6,P,P);







}
*/

void SegwayStateFeedbackController::getCofactor(double A[N][N], double temp[N][N], int p, int q, int n) 
{ 
    int i = 0, j = 0; 
  
    // Looping for each element of the matrix 
    for (int row = 0; row < n; row++) 
    { 
        for (int col = 0; col < n; col++) 
        { 
            //  Copying into temporary matrix only those element 
            //  which are not in given row and column 
            if (row != p && col != q) 
            { 
                temp[i][j++] = A[row][col]; 
  
                // Row is filled, so increase row index and 
                // reset col index 
                if (j == n - 1) 
                { 
                    j = 0; 
                    i++; 
                } 
            } 
        } 
    } 
} 
  
/* Recursive function for finding determinant of matrix. 
   n is current dimension of A[][]. */
double SegwayStateFeedbackController::determinant(double A[N][N], int n) 
{ 
    double D = 0.0; // Initialize result 
  
    //  Base case : if matrix contains single element 
    if (n == 1) 
        return A[0][0]; 
  
    double temp[N][N]; // To store cofactors 
  
    int sign = 1;  // To store sign multiplier 
  
     // Iterate for each element of first row 
    for (int f = 0; f < n; f++) 
    { 
        // Getting Cofactor of A[0][f] 
        getCofactor(A, temp, 0, f, n); 
        D += sign * A[0][f] * determinant(temp, n - 1); 
  
        // terms are to be added with alternate sign 
        sign = -sign; 
    } 
  
    return D; 
} 
  
// Function to get adjoint of A[N][N] in adj[N][N]. 
void SegwayStateFeedbackController::adjoint(double A[N][N],double adj[N][N]) 
{ 
    if (N == 1) 
    { 
        adj[0][0] = 1; 
        return; 
    } 
  
    // temp is used to store cofactors of A[][] 
    int sign = 1;
    double  temp[N][N]; 
  
    for (int i=0; i<N; i++) 
    { 
        for (int j=0; j<N; j++) 
        { 
            // Get cofactor of A[i][j] 
            getCofactor(A, temp, i, j, N); 
  
            // sign of adj[j][i] positive if sum of row 
            // and column indexes is even. 
            sign = ((i+j)%2==0)? 1: -1; 
  
            // Interchanging rows and columns to get the 
            // transpose of the cofactor matrix 
            adj[j][i] = (sign)*(determinant(temp, N-1)); 
        } 
    } 
} 
  
// Function to calculate and store inverse, returns false if 
// matrix is singular 
bool SegwayStateFeedbackController::inverse(double A[N][N], double inverse[N][N]) 
{ 
    // Find determinant of A[][] 
    double det = determinant(A, N); 
    if (det == 0.0) 
    { 
        cout << "Singular matrix, can't find its inverse"; 
        return false; 
    } 
  
    // Find adjoint 
    double adj[N][N]; 
    adjoint(A, adj); 
  
    // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
    for (int i=0; i<N; i++) {
        for (int j=0; j<N; j++) {
            inverse[i][j] = adj[i][j]/(det); 
 }} 
    return true; 
} 































void SegwayStateFeedbackController::update_speed(double speed, double speed_mes,double state_pitch,double state_pitch_rate) {



    // Compute the command
    double gain_pitch = gain[0] * state_pitch;
    double gain_pitchrate = gain[1] * state_pitch_rate;
    double gain_velocity = gain[2] * speed_mes;



    double vel = gain_pitch + gain_pitchrate + gain_velocity;




// Impose ramp limit on target speed
/* if ( (speed - target_speed) > (0.5 * ts * ramp_limit)) {
    target_speed = target_speed+Ramp * ts;
	//ROS_INFO(" t1: %f",target_speed);

  }

 else if ( (speed-target_speed) < (-0.5 * ts * ramp_limit)) {
    target_speed = target_speed-Ramp * ts;
	//ROS_INFO(" t2: %f",target_speed);

  }

 else {*/

    target_speed = speed;

 //}

    error = speed_mes - target_speed;

/*if(loop_counter %  50 == 0)

{

    // calculate delta_t


     delta_t = ros::Time::now() - prev_time;
      prev_time = ros::Time::now();
//}*/
 

 //  if (speed == 0 && error != 0) {
   // command = -vel;
// }

//   else {

//if(loop_counter % 50 == 0)
//	{
	delta_t = ros::Time::now() - prev_time;
      prev_time = ros::Time::now();
 

    integral_error = integral_error + Ki*delta_t.toSec()*error;
     
 

     loop_counter = 0;
 //}
   command = (-(integral_error)- vel) ;
     
//}
    
//loop_counter++;
speed_mesured = speed_mes;







}
void SegwayStateFeedbackController::update_omega(double omega_ref, double omega_mes) {


// Impose ramp limit on target speed
 if ((omega_ref - target_ang) > (0.5 * ts * ramp_limit)) {
    target_ang = target_ang+Ramp * ts;
  }

else if ((omega_ref-target_ang) < (-0.5 * ts * ramp_limit)) {
    target_ang = target_ang-Ramp * ts;
  }

 else {

    target_ang = omega_ref;

  }

    error_ang = omega_mes - target_ang;
 
   

      delta_t1 = ros::Time::now() - prev_time1;
      prev_time1 = ros::Time::now();
 


    integral_error_ang = integral_error_ang + Ki_ang*ts*error_ang;
   
 
    command_ang = Kp_ang*error_ang + integral_error_ang;


speed_ang_mes = omega_mes;
}


int SegwayStateFeedbackController::sgn(double v) {

  if (v < 0) return -1;
  if (v > 0) return 1;
 

}


double SegwayStateFeedbackController::antiWindup(double I) {

       if(!windup)

        {
            return I;
        }

       else if (I >= windupMax)
        {
            return windupMax;
        }

       else if (I <= -windupMax)
        {
            return -windupMax;
        }

       else
        {
            return I;
        }

}



double SegwayStateFeedbackController::getEffortCmd() {
    return command;
}

double SegwayStateFeedbackController::getEffort_Ang_Cmd() {
    return command_ang;
}

double SegwayStateFeedbackController::get_vel() {
    return x_estvel;
}


double SegwayStateFeedbackController::get_integral_error()

 { return integral_error;}

double SegwayStateFeedbackController::get_integral_error_ang()

 { return integral_error_ang;}


double SegwayStateFeedbackController::get_error()

 { return error;}

double SegwayStateFeedbackController::get_target_speed()

 { return target_speed;}

double SegwayStateFeedbackController::get_time()

 { return delta_t.toSec();}


double SegwayStateFeedbackController::get_pitch()

 { return x_estpitch;}

double SegwayStateFeedbackController::get_pitch_rate()

 { return x_estrate;}



void SegwayStateFeedbackController::print_state_linear()
{
ROS_INFO(" target_speed:%f,integral_error: %f, cmd:%f, error:%f, velocity:%f, time :%f",target_speed,integral_error, command,error,speed_mesured,delta_t.toSec());
}

void SegwayStateFeedbackController::print_state_angular()
{
ROS_INFO(" target_speed:%f,integral_error: %f, cmd_ang:%f, error:%f, angular_vel:%f, time :%f",target_ang,integral_error_ang, command_ang,error_ang,speed_ang_mes,delta_t.toSec());
}


void SegwayStateFeedbackController::setGain(std::vector<double> k) {
    if (k.size() != 3) {
        ROS_ERROR("Expecting a list of 3 elements representing a 3x1 matrix for K");
        return;
    }

    gain[0] = k[0];
    gain[1] = k[1];
    gain[2] = k[2];
}


void SegwayStateFeedbackController::setRampCoef(double ramp) {
Ramp = ramp;
}


void SegwayStateFeedbackController::setP(double KP) {
Kp_ang = KP;
}

void SegwayStateFeedbackController::setI(double KI) {
Ki_ang = KI;
}



void SegwayStateFeedbackController::setintegralCoef(double ki) {
Ki = ki;
}

void SegwayStateFeedbackController::setwindupCoef(double Windupmax)
{  windupMax = Windupmax;}



