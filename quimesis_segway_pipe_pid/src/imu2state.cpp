/*
 *  This node transforms the IMU data into the state data that we need.
 *
 *  Subscribes to:    /imu
 *  Pubslishes to:    /state/pitch
 *                    /state/pitch_rate
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>

#define GAZEBO

#ifdef GAZEBO
    #include <random>

    std::default_random_engine generator; 
   std::default_random_engine generator1;

std::normal_distribution<double> distribution(0.0,0.0);
std::normal_distribution<double> distribution1(0.0,0.0);
//std::normal_distribution<double> distribution(0.0,0.0004615);
//std::normal_distribution<double> distribution1(0.0,0.0058);
#endif



double state_pitch, state_pitch_rate,pitch_rate = 0;

ros::Publisher pub_pitch;
ros::Publisher pub_pitch_rate;
ros::Publisher pub_pitch_noise;
ros::Publisher pub_pitch_rate_noise;
ros::Subscriber sub_imu;


/*
 *  Callback to execute when we receive a message from the IMU
 *  The data is transformed to extract pitch and pitch_rate and those values
 *  are immediately re-published on the appropriate topics.
 *
 *  If we run a simulation in Gazebo, a little bit of noise is added to the IMU data
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& cmd)
{
    // Transform IMU data to pitch
    tf::Quaternion q(cmd->orientation.x, cmd->orientation.y, cmd->orientation.z, cmd->orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
pitch_rate = cmd->angular_velocity.y;
std_msgs::Float64 msg_pitch1, msg_pitch_rate1;
msg_pitch1.data = pitch;
msg_pitch_rate1.data = pitch_rate;
pub_pitch_noise.publish(msg_pitch1);
pub_pitch_rate_noise.publish(msg_pitch_rate1);

    #ifdef GAZEBO
        pitch = pitch + distribution(generator);
        pitch_rate = pitch_rate + distribution1(generator1);
    #endif

    state_pitch = -pitch+0.07-0.017+0.014;
    state_pitch_rate = -pitch_rate;

    std_msgs::Float64 msg_pitch, msg_pitch_rate;
    msg_pitch.data = state_pitch;
    msg_pitch_rate.data = state_pitch_rate;

    pub_pitch.publish(msg_pitch);
    pub_pitch_rate.publish(msg_pitch_rate);

    ROS_INFO("Pitch: %f, Pitch rate: %f", state_pitch, state_pitch_rate);


}


int main(int argc, char **argv)
{
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "imu2state");
    ros::NodeHandle nh;

    // Create 2 publishers to publish pitch and pitch rate
    pub_pitch = nh.advertise<std_msgs::Float64>("state/pitch", 1);
    pub_pitch_rate = nh.advertise<std_msgs::Float64>("state/pitch_rate", 1);

    pub_pitch_noise = nh.advertise<std_msgs::Float64>("state/pitch_noise", 1);
    pub_pitch_rate_noise = nh.advertise<std_msgs::Float64>("state/pitch_rate_noise", 1);

    // Create a subscriber for the IMU data
    sub_imu = nh.subscribe("imu", 1, imuCallback);


    ros::spin();
}
