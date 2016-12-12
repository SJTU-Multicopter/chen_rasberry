#include "ros/ros.h"  
#include "mavros_extras/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h" 


geometry_msgs::PoseStamped pos_msg;
geometry_msgs::TwistStamped vel_msg;

bool vel_mode = false;
bool pos_mode = false;

void chatterCallback_receive_pos_setpoint_local(const mavros_extras::PositionSetpoint &setpoint);
void chatterCallback_receive_vel_setpoint_local(const geometry_msgs::TwistStamped &setpoint);

int main(int argc, char **argv)  
{  
 
	ros::init(argc, argv, "publish_setpoints");

	ros::NodeHandle nh;  
	
	ros::Publisher offboard_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/offboard/setpoints", 5);  
	ros::Publisher offboard_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/offboard/velocity", 5);  
	ros::Subscriber setpoint_pos_sub = nh.subscribe("/offboard/position_setpoints_local", 5, chatterCallback_receive_pos_setpoint_local);
	ros::Subscriber setpoint_vel_sub = nh.subscribe("/offboard/velocity_setpoints_local", 5, chatterCallback_receive_vel_setpoint_local);

	ros::Rate loop_rate(16);
	while (ros::ok())  
	{   	
		if(pos_mode && !vel_mode)
		{
			if(pos_msg.pose.position.z > -1994) offboard_pos_pub.publish(pos_msg);
		}

		if(!pos_mode && vel_mode)
		{
			offboard_vel_pub.publish(vel_msg);
		}
		
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	
	return 0;  
}  

void chatterCallback_receive_pos_setpoint_local(const mavros_extras::PositionSetpoint &setpoint)
{
	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = setpoint.px;
	pos_msg.pose.position.y = setpoint.py;
	pos_msg.pose.position.z = setpoint.ph;
	pos_msg.pose.orientation.x = 0.0;
	pos_msg.pose.orientation.y = 0.0;
	pos_msg.pose.orientation.z = sin(setpoint.yaw/2);
	pos_msg.pose.orientation.w = cos(setpoint.yaw/2);

	pos_mode = true;
	vel_mode = false;
}

void chatterCallback_receive_vel_setpoint_local(const geometry_msgs::TwistStamped &setpoint)
{
	vel_msg.header.stamp = ros::Time::now();
	vel_msg.twist.linear.x = setpoint.twist.linear.x;;
    vel_msg.twist.linear.y = setpoint.twist.linear.y;
    vel_msg.twist.linear.z = setpoint.twist.linear.z;
    vel_msg.twist.angular.x = 0.0;
    vel_msg.twist.angular.y = 0.0;
    vel_msg.twist.angular.z = 0.0;

    pos_mode = false;
	vel_mode = true;
}
