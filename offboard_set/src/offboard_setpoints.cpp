#include "ros/ros.h"  
#include "geometry_msgs/PoseStamped.h" 
#include <mavros/SetPointLocal.h>
#include <sstream>  
#include <math.h>

void init_position();
void set_position(const mavros::SetPointLocal &setpoint);

bool offboard_ready = false;
bool setpoints_ready = false;
geometry_msgs::PoseStamped msg;

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "offboard_setpoints");

  ros::NodeHandle nh2;  

  init_position();
  
  ros::Publisher offboard_pub = nh2.advertise<geometry_msgs::PoseStamped>("offboard/setpoints", 5);  
  ros::Subscriber setpoint_sub = nh2.subscribe("/offboard/setpoints_local", 5, set_position);

  ros::Rate loop_rate(16);
  while (ros::ok())  
  {  
    if(setpoints_ready){
        //ROS_INFO("%f %f %f", msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
        offboard_pub.publish(msg);     
    }
    ros::spinOnce();  
  
    loop_rate.sleep();  
  }  
  
  
  return 0;  
}  


void init_position()
{
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 0.0;
}

void set_position(const mavros::SetPointLocal &setpoint)
{
  //ROS_INFO("Received");
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = setpoint.x;
  msg.pose.position.y = setpoint.y;
  msg.pose.position.z = setpoint.z;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = sin(setpoint.yaw/2);
  msg.pose.orientation.w = cos(setpoint.yaw/2);
  setpoints_ready = true;
}

