#include "ros/ros.h"  
#include <math.h>
#include "mavros_extras/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros/State.h"
#include "mavros_extras/ExtraFunctionReceiver.h"

#define Pi 3.1415926

mavros_extras::PositionSetpoint processed_setpoint;

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_mode(const mavros::State &msg);
void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg);
void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg);

bool offboard_ready = false;

float current_px = 0.0;
float current_py = 0.0;
float current_pz = 0.0;
float current_yaw = 0.0;

float new_setpoint_px = 0.0;
float new_setpoint_py = 0.0;
float new_setpoint_ph = 0.0;
float new_setpoint_yaw = 0.0;

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "process_setpoints");

  ros::NodeHandle nh;  
  
  ros::Publisher offboard_pub = nh.advertise<mavros_extras::PositionSetpoint>("offboard/setpoints_local", 2);  

  ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_raw", 2, chatterCallback_receive_setpoint_raw);
  ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 2,chatterCallback_local_position);
  ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,chatterCallback_mode);
  ros::Subscriber extrafunction_sub = nh.subscribe("/mavros/extra_function_receiver/extra_function_receiver", 1,chatterCallback_extra_function);

  
  ros::Rate loop_rate(10);
  while (ros::ok())  
  {  
  	if(new_setpoint_ph  > -1.5 && new_setpoint_ph < 0)
    {
      processed_setpoint.px = current_px;
      processed_setpoint.py = current_py;
      processed_setpoint.ph = current_pz;
      processed_setpoint.yaw = current_yaw;

    }
    else  //ph==-2 included, this will process in publish_setpoints.cpp
    {
      processed_setpoint.px = new_setpoint_px;
      processed_setpoint.py = new_setpoint_py;
      processed_setpoint.ph = new_setpoint_ph;
      processed_setpoint.yaw = new_setpoint_yaw;
    }
    
    offboard_pub.publish(processed_setpoint);
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  
  
  return 0;  
}  


void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg)
{
  new_setpoint_px = msg.px;
  new_setpoint_py = msg.py;
  new_setpoint_ph = msg.ph;
  new_setpoint_yaw = msg.yaw;
  ROS_INFO("yaw %f",new_setpoint_yaw);
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
  ROS_INFO("current_yaw %f",current_yaw);
}
void chatterCallback_mode(const mavros::State &msg)
{
  if(msg.mode=="OFFBOARD") offboard_ready=true;
  else offboard_ready=false;
}

void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg)
{
  ;
}
