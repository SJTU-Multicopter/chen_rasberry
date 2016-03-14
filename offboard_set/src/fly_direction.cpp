#include "ros/ros.h"
#include "mavros_extras/PositionSetpoint.h"
#include "mavros_extras/FlyDirection.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#define Pi 3.1415926

mavros_extras::FlyDirection direction_msg;
float set_h;
float set_x;
float set_y;
float current_px = 0.0;
float current_py = 0.0;
float current_pz = 0.0;
float current_yaw = 0.0;
float set_yaw = 0.0;

bool p_received = false;
bool setpoint_received = false;

void chatterCallback_receive_setpoint_local(const mavros_extras::PositionSetpoint &setpoint);
void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "fly_direction");

  ros::NodeHandle nh;

  ros::Publisher direction_pub = nh.advertise<mavros_extras::FlyDirection>("offboard/direction", 5);
  ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_local", 5, chatterCallback_receive_setpoint_local);
  ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 2,chatterCallback_local_position);
  ros::Rate loop_rate(4);
  while (ros::ok())
  {
    if(set_h > -1994)
    {
        set_yaw = - atan2(set_y-current_py,set_x-current_px);
        if(set_yaw < 0) set_yaw += 2*Pi;
        if(fabs(current_yaw - set_yaw) < Pi/4)  direction_msg.direction = 1;
        else if(fabs(current_yaw - set_yaw) > 3*Pi/4) direction_msg.direction = 2;
        else if(current_yaw - set_yaw > Pi/4 && current_yaw - set_yaw < 3*Pi/4) direction_msg.direction = 3;
        else direction_msg.direction = 4;

        direction_pub.publish(direction_msg);
    }
    else
    {
        direction_msg.direction = 0;
        direction_pub.publish(direction_msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

void chatterCallback_receive_setpoint_local(const mavros_extras::PositionSetpoint &setpoint)
{
  set_x = setpoint.px;
  set_y = setpoint.py;
  set_h = setpoint.ph;
  setpoint_received = true;
}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
  current_px = msg.pose.position.x;
  current_py = msg.pose.position.y;
  current_pz = msg.pose.position.z;

  float q2=msg.pose.orientation.x;
  float q1=msg.pose.orientation.y;
  float q0=msg.pose.orientation.z;
  float q3=msg.pose.orientation.w;
  //message.local_position.orientation.pitch = (asin(2*q0*q2-2*q1*q3 ))*57.3;
  //message.local_position.orientation.roll  = (atan2(2*q2*q3 + 2*q0*q1, 1-2*q1*q1-2*q2*q2))*57.3;
  current_yaw = (-atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1))+Pi;//North:0, south:Pi, East:Pi/2, West: Pi*3/2
  //ROS_INFO("current_yaw %f",current_yaw);
  p_received = true;
}
