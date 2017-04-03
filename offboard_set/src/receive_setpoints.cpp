#include "ros/ros.h"  
#include "../../../devel/include/mavros_msgs/OffboardRoutePoints.h"
#include "../../../devel/include/mavros_msgs/OffboardRoutePointsConfirm.h"
#include "../../../devel/include/mavros_msgs/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "../../../devel/include/mavros_msgs/State.h"
#include "std_msgs/Float32.h"
#include <math.h>
#define CLOSE_DIST 0.6  //m
#define Pi 3.14159265

void chatterCallback_route_points(const mavros_msgs::OffboardRoutePoints &msg);
void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_mode(const mavros_msgs::State &msg);
void chatterCallback_standard_height(const std_msgs::Float32 &msg);

mavros_msgs::PositionSetpoint setpoint;//(px,py,ph,yaw)
mavros_msgs::PositionSetpoint stop_setpoint;//(px,py,ph,yaw)
mavros_msgs::OffboardRoutePointsConfirm route_point_confirm;

bool near_bool(float x, float y);

float route_point[1002][3];//(x,y,h)
float route_yaw = 0.0;

//start px, py to correct setpoint, especially when the UAV get route points from GS while flying
float start_px = 0.0;
float start_py = 0.0;

float standard_height = 2.0;

float current_px = 0.0;
float current_py = 0.0;
float current_yaw = 0.0;
int total_num = -1;
int msg_seq = -1;
int close_counter = 0;
int send_counter = 0;

bool offboard_ready = false;
bool new_setpoint_writed = false;

int main(int argc, char **argv)  
{  
 
	ros::init(argc, argv, "receive_setpoints");

	ros::NodeHandle nh;  
	
        ros::Publisher routepoint_pub = nh.advertise<mavros_msgs::PositionSetpoint>("offboard/setpoints_raw", 2);
        ros::Publisher routepointconfirm_pub = nh.advertise<mavros_msgs::OffboardRoutePointsConfirm>("offboard_route_points_confirm", 2);

	ros::Subscriber setpoint_sub = nh.subscribe("/mavros/offboard_route_points_receiver/offboard_route_points_receiver", 5, chatterCallback_route_points);
	ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 2,chatterCallback_local_position);
	ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,chatterCallback_mode);
	ros::Subscriber standard_height_sub = nh.subscribe("/offboard/standard_height", 2,chatterCallback_standard_height);

	ros::Rate loop_rate(10);

	stop_setpoint.px = -1000.0;
	stop_setpoint.py = -1000.0;
	stop_setpoint.ph = -2000.0;
	stop_setpoint.yaw = route_yaw;

	while (ros::ok())  
	{  
		
		//send confirm message
		if(msg_seq >= 0)
		{
			route_point_confirm.px_1 = route_point[msg_seq][0];
			route_point_confirm.py_1 = route_point[msg_seq][1];
			route_point_confirm.ph_1 = route_point[msg_seq][2]; 
			route_point_confirm.px_2 = route_point[msg_seq+1][0];
			route_point_confirm.py_2 = route_point[msg_seq+1][1];
			route_point_confirm.ph_2 = route_point[msg_seq+1][2]; 
			route_point_confirm.seq = send_counter; //use this seq as the mark of fly position
			route_point_confirm.total = total_num;
			routepointconfirm_pub.publish(route_point_confirm);
		}
		
		//first send or resend data from groud station, reset send route point
		if(!offboard_ready && msg_seq < 1)
		{
			send_counter = 0;
			stop_setpoint.ph = -2000.0;
			routepoint_pub.publish(stop_setpoint); //ph = -2000.0, stop sending setpoint, reject offboard
		}
		//send new route point
		else if(new_setpoint_writed && (msg_seq - send_counter) >= -1 && send_counter <= total_num)
		{  		    
				routepoint_pub.publish(setpoint);
		}
		else 
		{   
			stop_setpoint.ph = -1000.0;
			routepoint_pub.publish(stop_setpoint); //ph = -1.0, stop the UAV by send local position as setpoint
		}
		
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	
	
	return 0;  
}  

void chatterCallback_route_points(const mavros_msgs::OffboardRoutePoints &msg)
{
	total_num = msg.total;
	msg_seq = msg.seq;
	route_point[msg_seq][0] = msg.px_1;
	route_point[msg_seq][1] = msg.py_1;
	route_point[msg_seq][2] = msg.ph_1;
	route_point[msg_seq+1][0] = msg.px_2;
	route_point[msg_seq+1][1] = msg.py_2;
	route_point[msg_seq+1][2] = msg.ph_2;
	route_yaw = msg.yaw;
}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
	if(offboard_ready)
	{
						 
			if(send_counter == 0) //initial point
			{
				setpoint.px = -1000.0; 
				setpoint.py = -1000.0;
				setpoint.ph = route_point[send_counter][2];
				setpoint.yaw = -120;  //<-100, mark the first take off point,wont get into trajactory generate in process_setpoints.cpp
				if(near_bool(setpoint.ph, standard_height))
					close_counter += 1;
				else {
					close_counter = 0;
				}       
			}
			else
			{
					if(near_bool(setpoint.px, msg.pose.position.x)&&near_bool(setpoint.py, msg.pose.position.y))
						close_counter += 1;
					else {
						close_counter = 0;
					}
			}

			if(close_counter >= 1){
					close_counter = 0;
						//set new route point
					send_counter += 1;
					setpoint.px = route_point[send_counter][0];
					setpoint.py = route_point[send_counter][1];
					setpoint.ph = route_point[send_counter][2];
					setpoint.yaw = route_yaw;
			}
			new_setpoint_writed = true;
	}
	else new_setpoint_writed = false;

	current_px = msg.pose.position.x;
	current_py = msg.pose.position.y;
	float q2=msg.pose.orientation.x; 
	float q1=msg.pose.orientation.y; 
	float q0=msg.pose.orientation.z; 
	float q3=msg.pose.orientation.w; 
	current_yaw = atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1) + Pi;//North:0, south:Pi, East:Pi/2, West: Pi*3/2 

}


void chatterCallback_mode(const mavros_msgs::State &msg)//模式
{
	if(msg.mode=="OFFBOARD") offboard_ready=true;
	else offboard_ready=false;
	//ROS_INFO("offboard ready!");
}

void chatterCallback_standard_height(const std_msgs::Float32 &msg)
{
	standard_height = msg.data;
}

bool near_bool(float x, float y)
{
	if(x-y< CLOSE_DIST && x-y> -CLOSE_DIST)
		return true;
	else return false;
}
