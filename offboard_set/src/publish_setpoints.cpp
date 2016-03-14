#include "ros/ros.h"  
#include "mavros_extras/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::PoseStamped msg;

void chatterCallback_receive_setpoint_local(const mavros_extras::PositionSetpoint &setpoint);

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "publish_setpoints");

  ros::NodeHandle nh;  
  
  ros::Publisher offboard_pub = nh.advertise<geometry_msgs::PoseStamped>("offboard/setpoints", 5);  
  ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_local", 5, chatterCallback_receive_setpoint_local);

  ros::Rate loop_rate(16);
  while (ros::ok())  
  {   	
    if(msg.pose.position.z > -1994) offboard_pub.publish(msg);
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  
  
  return 0;  
}  

void chatterCallback_receive_setpoint_local(const mavros_extras::PositionSetpoint &setpoint)
{
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = setpoint.px;
  msg.pose.position.y = setpoint.py;
  msg.pose.position.z = setpoint.ph;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = sin(setpoint.yaw/2);
  msg.pose.orientation.w = cos(setpoint.yaw/2);
}
