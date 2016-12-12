#include "ros/ros.h"  
#include <math.h>
#include "mavros_extras/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h" 
#include "mavros/State.h"
#include "mavros_extras/ExtraFunctionReceiver.h"
#include "mavros_extras/LaserDistance.h"
#include "Eigen/Dense"
#include "std_msgs/String.h"  
#include "std_msgs/Float32.h"

#define Pi 					3.141592653
#define NEAR_DISTANCE 		0.6
#define OBSTACLE_REGION 	6.0
#define SPEED_DOWN_REGION	2.0
#define LOOP_RATE_PLAN 		10

#define STATE_IDLE 			0
#define STATE_HOLDING 		1
#define STATE_TAKEOFF 		2
#define STATE_MOVING 		3
#define STATE_LANDING 		4

#define MOVING_SPEED_UP 	1
#define MOVING_SPEED_DOWN 	2
#define MOVING_NORMAL		0

using namespace Eigen;

void Callback_local_position(const geometry_msgs::PoseStamped &msg);
void Callback_mode(const mavros::State &msg);
void Callback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg);
void Callback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg);
void Callback_obstacle(const mavros_extras::LaserDistance &msg);  
void Callback_lidar(const lidar_driver::Lidar &msg);  
void rotate_2D(float yaw, const Vector2f& input, Vector2f& output); 
void rotate_3D(float yaw,  const Vector3f& input,  Vector3f& output);
bool isArrived(Vector3f& local, Vector3f& goal);

Vector3f local_pos(0.0,0.0);
Vector3f goal_pos(0.0,0.0); 
Vector3f obstacle_pos(0.0,0.0); 
Vector3f obstacle_pos_body(0.0,0.0);
Vector3f obstacle_pos_local(0.0,0.0);

float obstacle_distance = 0.0;  
float obstacle_angle = 0.0; 

float current_px = 0.0;
float current_py = 0.0;
float current_ph = 0.0;
float current_yaw = 0.0;
float lidar_height = 0.0;

float new_setpoint_px = 0.0;
float new_setpoint_py = 0.0;
float new_setpoint_ph = 0.0;
float new_setpoint_yaw = 0.0;
float start_px = 0.0;
float start_py = 0.0;
float start_ph = 0.0;
float start_yaw = 0.0;

std_msgs::Float32 standard_height;

//param for potential field
float k_att = 7.0;
float k_rep = 1.0;
float k_att_new = 24.0;
float speed = 1.0;

const float velocity = 0.5;

geometry_msgs::TwistStamped processed_vel_setpoint;
mavros_extras::PositionSetpoint processed_pos_setpoint;

Vector2f fly_direction(0.0,0.0); 

Vector2f vec1(0.0, 0.0);
Vector2f vec2(0.0, 0.0);

bool start_bool = true;
bool set_start_pos = false;
bool offboard_ready = false;
bool obstacle_avoid_enable = false;  
bool obstacle_avoid_height_enable = false; 
bool obstacle_avoid_auto_enable = false;  
bool laser_fly_height_enable = false;

//lidar running check
bool height_lidar_running = false;
bool obstacle_lidar_running = false;
bool height_lidar_check_flag = false;
bool obstacle_lidar_check_flag = false;

//state switch
int flight_state = 0;
int moving_state = 0;
bool arrived = false;
bool goal_init = false;

int main(int argc, char **argv)  
{  
	ros::init(argc, argv, "process_setpoints");
	ros::NodeHandle nh;  
	ros::Publisher offboard_pos_pub = nh.advertise<mavros_extras::PositionSetpoint>("offboard/position_setpoints_local", 2);  
	ros::Publisher offboard_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("offboard/velocity_setpoints_local", 2); 
	ros::Publisher standard_height_pub = nh.advertise<std_msgs::Float32>("offboard/standard_height", 2);

	ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_raw", 2, Callback_receive_setpoint_raw);
	ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 2,Callback_local_position);
	ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,Callback_mode);
	ros::Subscriber extrafunction_sub = nh.subscribe("/mavros/extra_function_receiver/extra_function_receiver", 1,Callback_extra_function);
	ros::Subscriber obstacle_sub = nh.subscribe("/laser_send",1,Callback_obstacle);
	ros::Subscriber lidar_sub = nh.subscribe("/lidar",1,Callback_lidar);

	ros::Rate loop_rate(LOOP_RATE_PLAN);

	while (ros::ok())  
	{  
		if(laser_fly_height_enable && height_lidar_running) standard_height.data = lidar_height;
		else standard_height.data = current_ph;
		standard_height_pub.publish(standard_height);

		if(new_setpoint_yaw < -100)
		// Takeoff 
		{
			if(start_bool) 
			{
				start_yaw = current_yaw;
				start_bool = false;
			}

			if(standard_height.data < 1.0)
			{
				processed_pos_setpoint.px = current_px;
				processed_pos_setpoint.py = current_py;
				processed_pos_setpoint.yaw = start_yaw;
			}else
			{
				if(!set_start_pos)
				{
					start_px = current_px;
					start_py = current_py;
					set_start_pos = true
				}
				processed_pos_setpoint.px = start_px;
				processed_pos_setpoint.py = start_py;
				processed_pos_setpoint.yaw = start_yaw;
			}
				
			//take off height set
			if(new_setpoint_ph - standard_height.data > 0.3) processed_pos_setpoint.ph = current_ph + 0.8;
			else if(standard_height.data - new_setpoint_ph > 0.3) processed_pos_setpoint.ph = current_ph - 0.4;
			else processed_pos_setpoint.ph = standard_height.data;

			//publish position setpoint
			offboard_pos_pub.publish(processed_pos_setpoint);

			flight_state = STATE_HOLDING;
			goal_init = false;
		}
		else if(new_setpoint_ph  > -1001.0 && new_setpoint_ph < -999.0)
		// ??????
		{
			processed_pos_setpoint.px = current_px;
			processed_pos_setpoint.py = current_py;
			processed_pos_setpoint.ph = current_ph;
			processed_pos_setpoint.yaw = current_yaw;
			start_bool = true;

			//publish position setpoint
			offboard_pos_pub.publish(processed_pos_setpoint);
		}
		else if(new_setpoint_ph < -1994)
		//??????
		{
			processed_setpoint.px = new_setpoint_px;
			processed_setpoint.py = new_setpoint_py;
			processed_setpoint.ph = new_setpoint_ph;
			processed_setpoint.yaw = new_setpoint_yaw;
			start_bool = true;

			//publish position setpoint
			offboard_pos_pub.publish(processed_pos_setpoint);
		}
		else
		{
			//initiate goal position
			if(!goal_init)
			{
				goal_pos(0) = new_setpoint_px;
				goal_pos(1) = new_setpoint_py;
				goal_pos(2) = 0.0;
				goal_init = true;
			}
			arrived = isArrived(local_pos,goal_pos);

			switch(flight_state)
			{
				case STATE_IDLE:
				{
					processed_pos_setpoint.px = current_px;
					processed_pos_setpoint.py = current_py;
					processed_pos_setpoint.ph = current_ph;
					processed_pos_setpoint.yaw = current_yaw;
					offboard_pos_pub.publish(processed_pos_setpoint);

					break;
				}
				case STATE_HOLDING:
				{
					ROS_INFO("Holding");
					for(int i = 10; ros::ok() && i > 0; --i){
						processed_pos_setpoint.px = current_px;
						processed_pos_setpoint.py = current_py;
						processed_pos_setpoint.ph = current_ph;
						processed_pos_setpoint.yaw = current_yaw;
						offboard_pos_pub.publish(processed_pos_setpoint);
						ros::spinOnce();
						rate.sleep();
					}

					//update goal_pos
					goal_pos(0) = new_setpoint_px;
					goal_pos(1) = new_setpoint_py;
					goal_pos(2) = 0.0;

					flight_state = STATE_MOVING;
					moving_state = MOVING_SPEED_UP;

					break;
				}
				case STATE_MOVING:
				{
					if(arrived)
					{
						processed_vel_setpoint.twist.linear.x = 0;
						processed_vel_setpoint.twist.linear.y = 0;
						processed_vel_setpoint.twist.linear.z = 0;
						processed_vel_setpoint.twist.angular.x = 0;
						processed_vel_setpoint.twist.angular.y = 0;
						processed_vel_setpoint.twist.angular.z = 0;
						offboard_vel_pub.publish(processed_vel_setpoint);

						flight_state = STATE_HOLDING;
					}else
					{
						switch(moving_state)
						{
							case MOVING_SPEED_UP:
							{
								for(int i = 0; ros::ok() && i < 10; i++){
									//ROS_INFO("Speed up");
									if((goal_pos - local_pos).norm() < SPEED_DOWN_REGION)
									{
										moving_state = MOVING_SPEED_DOWN;
										break;
									}else
									{
										Vector3f dirction = (goal_pos - local_pos).normalized();
										float vel = (float)i/10 * velocity;
										
										processed_vel_setpoint.twist.linear.x = vel * dirction(0);
										processed_vel_setpoint.twist.linear.y = vel * dirction(1);
										processed_vel_setpoint.twist.linear.z = 0.0;
										processed_vel_setpoint.twist.angular.x = 0.0;
										processed_vel_setpoint.twist.angular.y = 0.0;
										processed_vel_setpoint.twist.angular.z = 0.0;
										offboard_vel_pub.publish(processed_vel_setpoint);
										ros::spinOnce();
										rate.sleep();
									}
		
								}
								moving_state = MOVING_NORMAL;
								ROS_INFO("Normal fly");
								break;
							}
							case MOVING_SPEED_DOWN:
							{
								//ROS_INFO("Speed down");
								Vector3f dirction = (goal_pos - local_pos).normalized();
								float vel = ((goal_pos - local_pos).norm()) / SPEED_DOWN_REGION * velocity;
								
								processed_vel_setpoint.twist.linear.x = vel * dirction(0);
								processed_vel_setpoint.twist.linear.y = vel * dirction(1);
								processed_vel_setpoint.twist.linear.z = 0.0;
								processed_vel_setpoint.twist.angular.x = 0.0;
								processed_vel_setpoint.twist.angular.y = 0.0;
								processed_vel_setpoint.twist.angular.z = 0.0;
								offboard_vel_pub.publish(processed_vel_setpoint);
								ros::spinOnce();
								rate.sleep();

								break;
							}
							case MOVING_NORMAL:
							{
								if((goal_pos - local_pos).norm() < SPEED_DOWN_REGION)
								{
									moving_state = MOVING_SPEED_DOWN;
								}else
								{
									Vector3f dirction = (goal_pos - local_pos).normalized();
									float vel = velocity;
									
									processed_vel_setpoint.twist.linear.x = vel * dirction(0);
									processed_vel_setpoint.twist.linear.y = vel * dirction(1);
									processed_vel_setpoint.twist.linear.z = 0.0;
									processed_vel_setpoint.twist.angular.x = 0.0;
									processed_vel_setpoint.twist.angular.y = 0.0;
									processed_vel_setpoint.twist.angular.z = 0.0;
									offboard_vel_pub.publish(processed_vel_setpoint);
								}

								break;
							}
						}
								
					}

					break;
				}
				
				case STATE_LANDING:
				{
					break;
				}
			}

			// if((local_pos - goal_pos).norm() > NEAR_DISTANCE)
			// {
			// 	if(obstacle_distance < OBSTACLE_REGION && obstacle_distance > 0.1)
			// 	{
			// 		vec1 = (obstacle_pos - local_pos).normalized();
			// 		vec2 = (goal_pos - obstacle_pos).normalized();
			// 		if(vec1.dot(vec2)>0.9)
			// 		{
			// 			Vector2f goal_dir = goal_pos - local_pos;
			// 			Vector2f new_dir(0.0,0.0);
			// 			rotate_2D(Pi/2, goal_dir, new_dir);
			// 			if( new_dir.dot(obstacle_pos - local_pos) > 0.0 )
			// 			{
			// 				rotate_2D(-Pi/2, goal_dir, new_dir);
			// 			}
			// 			Vector2f force =k_att * (goal_pos - local_pos) + k_att_new * new_dir.normalized() + k_rep * (1/obstacle_distance - 1/OBSTACLE_REGION) * obstacle_distance * obstacle_distance * (local_pos - obstacle_pos) * (goal_pos - local_pos).norm() + 1/2 * k_rep * (1/obstacle_distance - 1/OBSTACLE_REGION) * (1/obstacle_distance - 1/OBSTACLE_REGION) * (local_pos - obstacle_pos);
			// 			fly_direction = force.normalized();
			// 		}else
			// 		{
			// 			Vector2f force = k_att * (goal_pos - local_pos) + k_rep *(1/obstacle_distance - 1/OBSTACLE_REGION) * obstacle_distance * obstacle_distance * (local_pos - obstacle_pos) * (goal_pos - local_pos).norm() + 1/2 * k_rep * (1/obstacle_distance - 1/OBSTACLE_REGION) * (1/obstacle_distance - 1/OBSTACLE_REGION) * (local_pos - obstacle_pos); 
			// 			fly_direction = force.normalized();
			// 		}
					
			// 	}else
			// 	{
			// 		fly_direction = (k_att * (goal_pos - local_pos)).normalized();
			// 	}

			// 	msg.twist.linear.x = fly_direction(0) * speed;
			//     msg.twist.linear.y = fly_direction(1) * speed;
			//     msg.twist.linear.z = 0.0;
			//     msg.twist.angular.x = 0.0;
			//     msg.twist.angular.y = 0.0;
			//     msg.twist.angular.z = 0.0;
			// }else
			// {
			// 	msg.twist.linear.x = 0.0;
			//     msg.twist.linear.y = 0.0;
			//     msg.twist.linear.z = 0.0;
			//     msg.twist.angular.x = 0.0;
			//     msg.twist.angular.y = 0.0;
			//     msg.twist.angular.z = 0.0;
			// }
		}

		
		ros::spinOnce();  
		loop_rate.sleep();  
	}
	return 0;  
}  

int float_near(float a, float b, float dif)
{
	float t = a-b;
	if(t < 0){
		t = -t;
	}
	if(t < dif){
		return 1;//is near
	}
	else{
		return 0;
	}
}
void Callback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg)
{
	new_setpoint_px = msg.px;
	new_setpoint_py = msg.py;
	new_setpoint_ph = msg.ph;
	new_setpoint_yaw = msg.yaw;
}

void Callback_local_position(const geometry_msgs::PoseStamped &msg)
{
	current_px = msg.pose.position.x;
	current_py = msg.pose.position.y;
	current_ph = msg.pose.position.z;
	local_pos(0) = msg.pose.position.x;  
	local_pos(1) = msg.pose.position.y;  
	local_pos(2) = 0.0;  

	float q2=msg.pose.orientation.x;
	float q1=msg.pose.orientation.y;
	float q0=msg.pose.orientation.z;
	float q3=msg.pose.orientation.w;
	current_yaw = atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1) + Pi;//North:0, south:Pi, East:Pi/2, West: Pi*3/2

}

void Callback_mode(const mavros::State &msg)
{
	if(msg.mode=="OFFBOARD") 
	{
		offboard_ready = true;
	}
	else 
	{
		offboard_ready = false;  
	}
}

void Callback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg)
{

}

//Subscribe obstacle msg by CJ
void Callback_obstacle(const mavros_extras::LaserDistance &msg)
{
	Vector3f direction;

	obstacle_distance = msg.min_distance / 100.0;
	obstacle_angle = msg.angle;

	direction = next_pos - local_pos;
	obstacle_pos_body(0) = obstacle_distance / 100.0 * cosf(obstacle_angle / 180.0 * Pi);
	obstacle_pos_body(1) = -obstacle_distance / 100.0 * sinf(obstacle_angle / 180.0 * Pi);
	obstacle_pos_body(2) = 0.0;
	rotate(current_yaw, obstacle_pos_body, obstacle_pos_local);
}

void Callback_lidar(const lidar_driver::Lidar &msg)
{

}

bool isArrived(Vector3f& local, Vector3f& goal)
{
	if((local - goal).norm() < NEAR_DISTANCE)
	{
		return true;
	}
	else
	{
		return false;
	}
}


//rotate function
void rotate_2D(float yaw,  const Vector2f& input,  Vector2f& output)
{
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	Matrix2f data;
	data(0,0) = cy;
	data(0,1) = -sy;
	data(1,0) = sy;
	data(1,1) = cy;

	output = data * input;
}

void rotate_3D(float yaw,  const Vector3f& input,  Vector3f& output)
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

