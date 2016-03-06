#include "ros/ros.h"  

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "receive_setpoints");

  ros::NodeHandle nh;  
  
  ros::Publisher offboard_pub = nh.advertise<geometry_msgs::PoseStamped>("offboard/setpoints", 1000);  
  ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_local", 500, set_position);

  ros::Rate loop_rate(16);
  while (ros::ok())  
  {  
  	
    
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  
  
  return 0;  
}  
