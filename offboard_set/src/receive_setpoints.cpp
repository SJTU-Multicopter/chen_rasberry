#include "ros/ros.h"  
#include "mavros_extras/OffboardRoutePoints.h"
#include "mavros_extras/OffboardRoutePointsConfirm.h"
#include "mavros_extras/PositionSetpoint.h"
#include "mavros_extras/ExtraFunctionReceiver.h"
#include "mavros_extras/LaserDistance.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros/State.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include "Eigen/Dense"
#define CLOSE_DIST 0.6  //m
#define Pi 3.14159265
using namespace Eigen;

void chatterCallback_route_points(const mavros_extras::OffboardRoutePoints &msg);
void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_mode(const mavros::State &msg);
void chatterCallback_standard_height(const std_msgs::Float32 &msg);
void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg); //add by CJ
void chatterCallback_obstacle(const mavros_extras::LaserDistance &msg);  //add by CJ
void rotate(float yaw, const Vector3f& input, Vector3f& output);   //add by CJ
void obstacle_avoid_trajectory_generation(const Vector3f& current_pos, const Vector3f& next_pos, Matrix<float, 4, 2> trajectory_matrix);

mavros_extras::PositionSetpoint setpoint;//(px,py,ph,yaw)
mavros_extras::PositionSetpoint stop_setpoint;//(px,py,ph,yaw)
mavros_extras::OffboardRoutePointsConfirm route_point_confirm;

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

bool obstacle_avoid_enable = false;  //add by CJ
bool obstacle_avoid_height_enable = false;  //add by CJ
bool obstacle_avoid_auto_enable = false;  //add by CJ
bool auto_avoid_processing = false; //add by CJ
bool manual_avoid = false;  //add by CJ 
bool fly_processing = false;  //add by CJ
bool fly_direction_enable = false; //add by CJ
bool disturb = false;  //add by  CJ
bool obstacle = false;  //add by CJ
bool switch_offboard = false; //add by CJ
bool height_lidar_running = false; //add by CJ
bool obstacle_lidar_running = false; //add by CJ
bool height_lidar_check_flag = false; //add by CJ
bool obstacle_lidar_check_flag = false; //add by CJ
int timer_counter = 0; //add by CJ
int disturb_counter = 0; //add by CJ
int lidar_counter = 0;//add by CJ
float laser_height_last = 0.0;//add by CJ
float laser_height = 3.0;//add by CJ
int fly_direction = 0; //add by CJ
int auto_avoid_count = 0;  //add by CJ
float obstacle_distance = 6.0;  //add by CJ
float obstacle_angle = 0.0;  //add by CJ
float obstacle_distance_prev = 6.0;  //add by CJ
float obstacle_angle_prev = 0.0;  //add by CJ
float laser_distance = 6.0;  //add by CJ
float laser_angle = 0.0;  //add by CJ
float laser_distance_pre = 6.0;  //add by CJ
float laser_angle_pre = 0.0;  //add by CJ
Vector3f next_pos(0.0,0.0,0.0);  //add by CJ
Vector3f local_pos(0.0,0.0,0.0);  //add by CJ
Vector3f body_pos(0.0,0.0,0.0);  //add by CJ
Vector3f local_pos_stop(0.0,0.0,0.0);  //add by CJ
Vector3f body_pos_stop(0.0,0.0,0.0);  //add by CJ
Matrix<float, 4, 2> obstacle_avoid_trajectory;  //add by CJ

int main(int argc, char **argv)  
{  
 
	ros::init(argc, argv, "receive_setpoints");

	ros::NodeHandle nh;  
	
	ros::Publisher routepoint_pub = nh.advertise<mavros_extras::PositionSetpoint>("offboard/setpoints_raw", 2); 
	ros::Publisher routepointconfirm_pub = nh.advertise<mavros_extras::OffboardRoutePointsConfirm>("offboard_route_points_confirm", 2); 

	ros::Subscriber setpoint_sub = nh.subscribe("/mavros/offboard_route_points_receiver/offboard_route_points_receiver", 5, chatterCallback_route_points);
	ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 2,chatterCallback_local_position);
	ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,chatterCallback_mode);
	ros::Subscriber standard_height_sub = nh.subscribe("/offboard/standard_height", 2,chatterCallback_standard_height);

	ros::Subscriber extrafunction_sub = nh.subscribe("/mavros/extra_function_receiver/extra_function_receiver", 1,chatterCallback_extra_function);
	ros::Subscriber obstacle_sub = nh.subscribe("/laser_send",1,chatterCallback_obstacle);
	ros::Subscriber crop_distance_sub = nh.subscribe("/crop_dist",1,chatterCallback_crop_distance);

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
		
		//<add by CJ
		if(manual_avoid){
			rotate(-current_yaw, local_pos, body_pos);
			body_pos_stop(0) = body_pos(0) - (300.0 - obstacle_distance) / 100.0f * cosf(obstacle_angle / 180.0 * Pi);
			body_pos_stop(1) = body_pos(1) + (300.0 - obstacle_distance) / 100.0f * sinf(obstacle_angle / 180.0 * Pi);
			rotate(current_yaw, body_pos_stop, local_pos_stop);
			// while(offboard_ready && ros::ok() && !disturb  && !obstacle)
			// {
			// 	stop_setpoint.ph = -1000.0;
			// 	routepoint_pub.publish(stop_setpoint); 
			// 	ros::spinOnce();  
			// 	loop_rate.sleep();
			// }
			while(offboard_ready && ros::ok() && obstacle)
			{
				stop_setpoint.px = local_pos_stop(0);
				stop_setpoint.py = local_pos_stop(1);
				stop_setpoint.ph = current_ph;
				stop_setpoint.yaw = current_yaw; 
				routepoint_pub.publish(stop_setpoint);
				ros::spinOnce();  
				loop_rate.sleep();
			}
		//add by CJ >

		}else{
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
		}
		
		
		ros::spinOnce();  
		loop_rate.sleep();  
	}  
	
	
	return 0;  
}  

void chatterCallback_route_points(const mavros_extras::OffboardRoutePoints &msg)
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

			// <add by CJ
			if(auto_avoid_processing){
				if(auto_avoid_count == 0){
					obstacle_avoid_trajectory << 0.0, 0.0,     
							0.0, 0.0, 
							0.0, 0.0, 
							0.0, 0.0;
					obstacle_avoid_trajectory_generation(local_pos, next_pos, obstacle_avoid_trajectory);
					auto_avoid_count++;
				}
				if(auto_avoid_count == 1){
					if(!fly_processing){
						setpoint.px = obstacle_avoid_trajectory(1,0);
						setpoint.py = obstacle_avoid_trajectory(1,1);
						setpoint.ph = route_point[send_counter][2];
						setpoint.yaw = route_yaw;
						fly_processing = true;
					}
					else{
						if(near_bool(setpoint.px, msg.pose.position.x) && near_bool(setpoint.px, msg.pose.position.x)){
							auto_avoid_count++;
							fly_processing = false;
						}
					}  
				}
				if(auto_avoid_count == 2){
					if(!fly_processing){
						setpoint.px = obstacle_avoid_trajectory(2,0);
						setpoint.py = obstacle_avoid_trajectory(2,1);
						setpoint.ph = route_point[send_counter][2];
						setpoint.yaw = route_yaw;
						fly_processing = true;
					}else{
						if(near_bool(setpoint.px, msg.pose.position.x) && near_bool(setpoint.px, msg.pose.position.x)){
							auto_avoid_count++;
							fly_processing = false;
						}
					}  
				}
				if(auto_avoid_count == 3){
					if(!fly_processing){
						setpoint.px = obstacle_avoid_trajectory(3,0);
						setpoint.py = obstacle_avoid_trajectory(3,1);
						setpoint.ph = route_point[send_counter][2];
						setpoint.yaw = route_yaw;
						fly_processing = true;
					}else{
						if(near_bool(setpoint.px, msg.pose.position.x) && near_bool(setpoint.px, msg.pose.position.x)){
							auto_avoid_count++;
							fly_processing = false;
						}
					} 
				}
				if(auto_avoid_count == 4){
					auto_avoid_processing = false;    
					auto_avoid_count = 0;
				}
			//add by CJ>

			}else{
				close_counter = 0;
				//set new route point
				send_counter += 1;
				setpoint.px = route_point[send_counter][0];
				setpoint.py = route_point[send_counter][1];
				setpoint.ph = route_point[send_counter][2];
				setpoint.yaw = route_yaw;
			}
			
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

	local_pos(0) = msg.pose.position.x;  //add by CJ
	local_pos(1) = msg.pose.position.y;  //add by CJ
	local_pos(2) = 0.0;  //add by CJ
	next_pos(0) = route_point[send_counter][0];    //add by CJ
	next_pos(1) = route_point[send_counter][1];    //add by CJ
	next_pos(2) = 0.0;       //add by CJ

}


void chatterCallback_mode(const mavros::State &msg)//模式
{
	if(msg.mode=="OFFBOARD") 
	{
		offboard_ready = true;
		if(!switch_offboard){
			switch_offboard = true;
			obstacle = false;

			obstacle_distance_prev = 6.0;
			obstacle_angle_prev = 0.0;
			obstacle_distance = 6.0;
			obstacle_angle = 0.0;
		}
	}
	else{
		offboard_ready = false;
		switch_offboard = false;
	} 

	//use as timer, 1Hz
	timer_counter += 1;
	if(timer_counter > 5)
	{
		timer_counter = 0;

		if(height_lidar_check_flag) 
		{
			height_lidar_check_flag = false;
			height_lidar_running = true;
		}
		else
		{
			height_lidar_running = false;
		}

		if(obstacle_lidar_check_flag)
		{
			obstacle_lidar_check_flag = false;
			obstacle_lidar_running = true;
		}
		else
		{
			obstacle_lidar_running = false;
		} 
	}
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

//Subscribe crop distance msg by CJ
void chatterCallback_crop_distance(const std_msgs::Float32 &msg)
{
	if(msg.data <= -1.5)
	{
		obstacle_avoid_height_enable = true;
	}else
	{
		obstacle_avoid_height_enable = false;
	}
	laser_height = -msg.data;

	lidar_counter += 1;
	if(lidar_counter > 20)
	{
		if(fabs(laser_height_last - laser_height)<0.00001) height_lidar_running = false;
		else height_lidar_running = true;

		if(fabs(laser_height-6.0)<0.01)  height_lidar_running = true;

		laser_height_last = laser_height;
		lidar_counter = 0;
	}
	height_lidar_check_flag = true;
}

//Subscribe obstacle msg by CJ
void chatterCallback_obstacle(const mavros_extras::LaserDistance &msg)
{
	Vector3f obstacle_pos_body;
	Vector3f obstacle_pos_local;
	Vector3f direction;

	laser_distance_pre = laser_distance;
	laser_angle_pre = laser_angle;
	laser_distance = msg.min_distance;
	laser_angle = msg.angle;


	direction = next_pos - local_pos;
	obstacle_pos_body(0) = laser_distance / 100.0 * cosf(laser_angle / 180.0 * Pi);
	obstacle_pos_body(1) = -laser_distance / 100.0 * sinf(laser_angle / 180.0 * Pi);
	obstacle_pos_body(2) = 0.0;
	rotate(current_yaw, obstacle_pos_body, obstacle_pos_local);
	if(direction.dot(obstacle_pos_local) > 0) fly_direction_enable = true;
	else fly_direction_enable = false;


	if(laser_distance > 90.0 && laser_distance < 400.0)
	{
		obstacle_distance_prev = obstacle_distance;
		obstacle_angle_prev = obstacle_angle;
		obstacle_distance = laser_distance;
		obstacle_angle = laser_angle;

		if((obstacle_distance_prev - obstacle_distance > 250.0) || fabs(obstacle_angle - obstacle_angle_prev) > 60.0f)
		{
			disturb = true;
		}else
		{
			disturb = false;
			obstacle = true;
		}

		if(obstacle_avoid_enable && obstacle_avoid_height_enable && obstacle_avoid_auto_enable && !auto_avoid_processing && fly_direction_enable && obstacle_lidar_running)  
		{
			auto_avoid_processing = true;
		}

		if(obstacle_avoid_enable && obstacle_avoid_height_enable && !obstacle_avoid_auto_enable && fly_direction_enable && obstacle_lidar_running)
		{
			manual_avoid = true;
		}
		else{
			manual_avoid = false;
		} 
	}else
	{
		manual_avoid = false;
	}

	obstacle_lidar_check_flag = true;
}

//rotate function
void rotate(float yaw,  const Vector3f& input,  Vector3f& output)
{
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	Matrix3f data;
	data(0,0) = cy;
	data(0,1) = -sy;
	data(0,2) = 0.0;
	data(1,0) = sy;
	data(1,1) = cy;
	data(1,2) = 0.0;
	data(2,0) = 0.0;
	data(2,1) = 0.0;
	data(2,2) = 1.0;

	output = data * input;
}

void obstacle_avoid_trajectory_generation(const Vector3f& current_position, const Vector3f& next_position, Matrix<float, 4, 2> trajectory_matrix)
{
	Vector3f obstacle_pos_body;
	Vector3f obstacle_pos_local;
	Vector3f direction;
	Vector3f n_vector;

	obstacle_pos_body(0) = obstacle_distance / 100.0 * cosf(obstacle_angle / 180.0 * Pi);
	obstacle_pos_body(1) = -obstacle_distance / 100.0 * sinf(obstacle_angle / 180.0 * Pi);
	rotate(current_yaw, obstacle_pos_body, obstacle_pos_local);

	direction = next_position - current_position;
	direction = direction.normalized();
	n_vector(0) = direction(1);
	n_vector(1) = -direction(0);
	n_vector = n_vector.normalized();
	if(n_vector.dot(obstacle_pos_local) >= 0) n_vector = -n_vector;

	trajectory_matrix(0,0) = current_position(0);
	trajectory_matrix(0,1) = current_position(1);
	trajectory_matrix(1,0) = current_position(0) + 2 * n_vector(0);
	trajectory_matrix(1,1) = current_position(1) + 2 * n_vector(1);
	trajectory_matrix(2,0) = trajectory_matrix(1,0) + 4.0 * direction(0);
	trajectory_matrix(2,1) = trajectory_matrix(1,1) + 4.0 * direction(1);
	trajectory_matrix(3,0) = current_position(0) + 4.0 * direction(0);
	trajectory_matrix(3,1) = current_position(1) + 4.0 * direction(1);
}
