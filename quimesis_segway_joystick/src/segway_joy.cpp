#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopSegway
{
public:
  TeleopSegway();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopSegway::TeleopSegway():
  linear_(3),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopSegway::joyCallback, this);

}

void TeleopSegway::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  #ifdef GAZEBO
    twist.angular.z = -a_scale_*joy->axes[angular_]; // - for the joystick the left is negative
    twist.linear.x = l_scale_*joy->axes[linear_];
  #else
    twist.angular.z = a_scale_*joy->axes[angular_]; // - for the joystick the left is negative
    twist.linear.x = -l_scale_*joy->axes[linear_];
  #endif
  vel_pub_.publish(twist);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_node");
  ROS_INFO("Starting setvelocity publisher");

  TeleopSegway teleop_segway;
  ros::spin();


}
