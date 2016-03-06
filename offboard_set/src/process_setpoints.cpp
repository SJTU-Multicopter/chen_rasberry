#include "ros/ros.h"  
#include "mavros_extras/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros/State.h"
#include <mavros_extras/ExtraFunctionReceiver.h>


mavros_extras::PositionSetpoint processed_setpoint;

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_mode(const mavros::State &msg);
void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg);
void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg);

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
  	
    
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  
  
  return 0;  
}  


void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg)
{
  ;
}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
  ;
}
void chatterCallback_mode(const mavros::State &msg)
{
  ;
}

void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg)
{
  ;
}