#include "ros/ros.h"  
#include <mavros_extras/FieldSize.h>
#include <mavros_extras/FieldSizeConfirm.h>
#include <math.h>
#include <sstream>  
#include <ros/console.h>

bool f_equal(float x, float y);
void chatterCallback_Field_Size_Confirm(const mavros_extras::FieldSizeConfirm &msg);

bool send_ok = false;

mavros_extras::FieldSize size_msg;

int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "field_size_set");  

  size_msg.length = 10.0f;
  size_msg.width = 3.0f;
  size_msg.height = 4.0f;
  size_msg.times = 6;

  ros::NodeHandle n;  
  ros::Publisher field_size_pub = n.advertise<mavros_extras::FieldSize>("field_size_set", 500);  
  ros::Subscriber confirm_sub = n.subscribe("/mavros/field_size_confirm_receiver/field_size_confirm_receiver", 200,chatterCallback_Field_Size_Confirm);

  ros::Rate loop_rate(1);  
  

  while (ros::ok()&& !send_ok)  
  {  
    field_size_pub.publish(size_msg);    
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  
  ROS_INFO("Field Size Sent Correctly!");
  
  return 0;  
}  

void chatterCallback_Field_Size_Confirm(const mavros_extras::FieldSizeConfirm &msg)
{
  if(f_equal(msg.length,size_msg.length)&&f_equal(msg.width,size_msg.width)&&f_equal(msg.height,size_msg.height)&&msg.times==size_msg.times)
    send_ok = true;
}

bool f_equal(float x, float y)
{
  if(fabs(x-y)<0.01) return true;
  else return false; 
}