#include "ros/ros.h"  
#include <math.h>
#include "mavros_extras/PositionSetpoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros/State.h"
#include "mavros_extras/ExtraFunctionReceiver.h"
#include "mavros_extras/LaserDistance.h"
#include "mavros_extras/FlyDirection.h"
#include "Eigen/Dense"
#include "std_msgs/String.h"   //new
#include "std_msgs/Float32.h"
#define Pi 3.1415926
#define LOOP_RATE_PLAN 10
using namespace Eigen;

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_mode(const mavros::State &msg);
void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg);
void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg);
void chatterCallback_crop_distance(const std_msgs::Float32 &msg);  //add by CJ

float posPlan(float max_jerk, float max_acc, float t, 
	int stage, const VectorXf& nodes_time, 
	const VectorXf& nodes_vel, const VectorXf& nodes_pos);

float velPlan(float max_jerk, float max_acc, float t, 
	int stage, const VectorXf& nodes_time, const VectorXf& nodes_vel);

float accPlan(float max_jerk, float max_acc, float t, 
	int stage, const VectorXf& nodes_time);

float jerkPlan(float max_jerk, int stage);

int trapezoidalTraj(float start_pos, float ended_pos, 
	float MAX_v, float MAX_pitch_deg, float MAX_j,
	VectorXf& nodes_time, 
	VectorXf& nodes_vel, 
	VectorXf& nodes_pos,
	float* max_acc);

void trajectory_Paras_generation_i(int num, float p0, float pf, float T, Matrix<float, 3, 3>& Paras_matrix);//num 0,1,2 reoresents  x, y, z
float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t, float p0);
float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float p_simple_calculate(int num, float t, float vel, float p0);
float distance(float x0, float y0, float x1, float y1);



bool offboard_ready = false;
bool laser_fly_height_enable = false;
bool height_lidar_running = false;
bool height_lidar_check_flag = false;

int lidar_counter = 0;
float laser_height_last = 0.0;
float laser_height = 3.0;

float current_px = 0.0;
float current_py = 0.0;
float current_ph = 0.0;
float current_yaw = 0.0;

float new_setpoint_px = 0.0;
float new_setpoint_py = 0.0;
float new_setpoint_ph = 0.0;
float new_setpoint_yaw = 0.0;

float start_pos[2] = {0.0,0.0};
float ended_pos[2] = {0.0,0.0};
float start_ph = 0.0;
float start_yaw = 0.0;
bool start_bool = true;

bool different_sp_rcv = false;

mavros_extras::PositionSetpoint processed_setpoint;
std_msgs::Float32 standard_height;
mavros_extras::LaserDistance record_values;

int main(int argc, char **argv)  
{  
	
	ros::init(argc, argv, "process_setpoints");

	ros::NodeHandle nh;  
	
	ros::Publisher offboard_pub = nh.advertise<mavros_extras::PositionSetpoint>("offboard/setpoints_local", 2);  
	ros::Publisher standard_height_pub = nh.advertise<std_msgs::Float32>("offboard/standard_height", 2);
	ros::Publisher record_paras_pub = nh.advertise<mavros_extras::LaserDistance>("offboard/record",2);

	ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_raw", 2, chatterCallback_receive_setpoint_raw);
	ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 2,chatterCallback_local_position);
	ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 1,chatterCallback_mode);
	ros::Subscriber extrafunction_sub = nh.subscribe("/mavros/extra_function_receiver/extra_function_receiver", 1,chatterCallback_extra_function);
	ros::Subscriber crop_distance_sub = nh.subscribe("/crop_dist",1,chatterCallback_crop_distance);
	ros::Subscriber fly_direction_sub = nh.subscribe("/offboard/direction", 1,chatterCallback_fly_direction);
	
	ros::Rate loop_rate(LOOP_RATE_PLAN);

	float MAX_v = 3.0, MAX_pitch_deg = 20.0, MAX_j = 2.5;
	float max_a = 0;

	int method = 1;
	float avrg_vel = 1.0;
	float time2fly = 0.0;
	float current_t = 0.0;
	float avrg_vx = 0.0;
	float avrg_vy = 0.0;
	float theta = 0.0;
	int stage = 1;
	VectorXf nodes_t = VectorXf::Zero(8);
	VectorXf nodes_p = VectorXf::Zero(8);
	VectorXf nodes_v = VectorXf::Zero(8);

	Matrix<float, 3, 3> Paras_matrix(3,3);
	while (ros::ok())  
	{  
		//tell the standard height, measured by barometer or rplidar
		if(laser_fly_height_enable && height_lidar_running) standard_height.data = laser_height;
		else standard_height.data = current_ph;
		standard_height_pub.publish(standard_height);

		record_values.min_distance = start_pos[0];
		record_values.angle = start_pos[1];
		record_values.laser_x = ended_pos[0];
		record_values.laser_y = ended_pos[1];
		record_paras_pub.publish(record_values);

		if(new_setpoint_yaw < -100)
		{
			if(start_bool) 
			{
				start_yaw = current_yaw;
				start_bool = false;
			}

			processed_setpoint.px = current_px;
			processed_setpoint.py = current_py;
			processed_setpoint.yaw = start_yaw;

			start_pos[0] = current_px;
			start_pos[1] = current_py;
			ended_pos[0] = current_px;
			ended_pos[1] = current_py;
			//take off height set
 
			if(new_setpoint_ph - standard_height.data > 0.3) processed_setpoint.ph = current_ph + 0.8;
			else if(standard_height.data - new_setpoint_ph > 0.3) processed_setpoint.ph = current_ph - 0.4;
			else processed_setpoint.ph = standard_height.data;

		}
		else if(new_setpoint_ph  > -1001.0 && new_setpoint_ph < -999.0)
		{
			processed_setpoint.px = current_px;
			processed_setpoint.py = current_py;
			processed_setpoint.ph = current_ph;
			processed_setpoint.yaw = current_yaw;

			start_bool = true;
		}
		else if(new_setpoint_ph < -1994)
		{
			processed_setpoint.px = new_setpoint_px;
			processed_setpoint.py = new_setpoint_py;
			processed_setpoint.ph = new_setpoint_ph;
			processed_setpoint.yaw = new_setpoint_yaw;
			start_bool = true;
		}
		else//ph==-2 included, this will process in publish_setpoints.cpp
		{
			if(different_sp_rcv){//traj init
				different_sp_rcv = false;

				float length = distance(start_pos[0], start_pos[1], new_setpoint_px, new_setpoint_py);
				
				time2fly = length/avrg_vel;
				avrg_vx = (ended_pos[0] - current_px) / time2fly;
				avrg_vy = (ended_pos[1] - current_py) / time2fly;
				
				current_t = 0;     

				stage = 1;
				nodes_t = VectorXf::Zero(8);
				nodes_p = VectorXf::Zero(8);
				nodes_v = VectorXf::Zero(8);
				float dx = ended_pos[0] - start_pos[0];
				float dy = ended_pos[1] - start_pos[1];
				theta = atan2(dy, dx);
				method = trapezoidalTraj(0, length, MAX_v, MAX_pitch_deg, MAX_j,nodes_t, nodes_v, nodes_p, &max_a);
				std::cout << "\nnodes_t:\n" << nodes_t << std::endl;
				std::cout << "\nnodes_p:\n" << nodes_p << std::endl;
				std::cout << "\nnodes_v:\n" << nodes_v << std::endl;
				if(method == 1){//non const vel
					trajectory_Paras_generation_i(0, start_pos[0], new_setpoint_px,time2fly, Paras_matrix);
					trajectory_Paras_generation_i(1, start_pos[1], new_setpoint_py,time2fly, Paras_matrix);

				}
			}//end of init
			if(method == 1){//non const vel
				if(current_t < time2fly){
					processed_setpoint.px = p_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),current_t, start_pos[0]); 
					processed_setpoint.py = p_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),current_t, start_pos[1]);
				}
				else{
					processed_setpoint.px = new_setpoint_px;
					processed_setpoint.py = new_setpoint_py;
				}
			}
			else{//trapezoidal
				
				if(stage < 8){
					if(current_t > nodes_t(stage)){
						stage++;
					}
					// float j = jerkPlan(MAX_j, stage);
					// float a = accPlan(MAX_j, max_a, current_t, stage, nodes_t);
					// float v = velPlan(MAX_j, max_a, current_t, stage, nodes_t, nodes_v);
					float p = posPlan(MAX_j, max_a, current_t, stage, nodes_t, nodes_v, nodes_p);
					// float j_x = j * cos(theta);
					// float j_y = j * sin(theta);
//          float a_x = a * cos(theta);
//          float a_y = a * sin(theta);
//          float v_x = v * cos(theta);
//          float v_y = v * sin(theta);
					float p_x = p * cos(theta) + start_pos[0];
					float p_y = p * sin(theta) + start_pos[1];
//          ROS_INFO("t: %f stage: %d\njx: %f ax: %f vx: %f px: %f\njy: %f ay: %f vy: %f py: %f\n", 
//            current_t, stage, 
//            j_x, a_x, v_x, p_x, 
//            j_y, a_y, v_y, p_y);
					processed_setpoint.px = p_x;
					processed_setpoint.py = p_y;
				}
				else{
					processed_setpoint.px = new_setpoint_px;
					processed_setpoint.py = new_setpoint_py;

				}
			}
//      ROS_INFO("method: %d x_sp: %f y_sp: %f", method, processed_setpoint.px, processed_setpoint.py);
//      processed_setpoint.px = new_setpoint_px;
//      processed_setpoint.py = new_setpoint_py;
//      processed_setpoint.ph = new_setpoint_ph;
//      processed_setpoint.yaw = new_setpoint_yaw;
			/*set height*/
			if(laser_fly_height_enable && height_lidar_running) processed_setpoint.ph = new_setpoint_ph - laser_height + current_ph ;
			else processed_setpoint.ph = new_setpoint_ph;

			processed_setpoint.yaw = new_setpoint_yaw;

		}//end of if(new_setpoint_ph  > -1.5 && new_setpoint_ph < 0)

		current_t += 1.0 / LOOP_RATE_PLAN;

		offboard_pub.publish(processed_setpoint);
		ros::spinOnce();  
		loop_rate.sleep();  
	}
	return 0;  
}  
float distance(float x0, float y0, float x1, float y1)
{
	return (sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)));
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
void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg)
{
	if(msg.px > -999.0) //if(offboard_ready)
	{
		if(float_near(msg.px, new_setpoint_px, 0.05) && float_near(msg.py, new_setpoint_py, 0.05))// && float_near(msg.ph, new_setpoint_ph, 0.05))
		{
			;
		}
		else{
			start_pos[0] = ended_pos[0];
			start_pos[1] = ended_pos[1];
			ended_pos[0] = msg.px;
			ended_pos[1] = msg.py;
			//start_ph = current_ph;
			//start_yaw = current_yaw;
			different_sp_rcv = true;
		}
	}
	else
	{
		start_pos[0] = current_px;
		start_pos[1] = current_py;
		ended_pos[0] = current_px;
		ended_pos[1] = current_py; 
	}

	new_setpoint_px = msg.px;
	new_setpoint_py = msg.py;

	new_setpoint_ph = msg.ph;
	new_setpoint_yaw = msg.yaw;

}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
	current_px = msg.pose.position.x;
	current_py = msg.pose.position.y;
	current_ph = msg.pose.position.z;

	float q2=msg.pose.orientation.x;
	float q1=msg.pose.orientation.y;
	float q0=msg.pose.orientation.z;
	float q3=msg.pose.orientation.w;
	//message.local_position.orientation.pitch = (asin(2*q0*q2-2*q1*q3 ))*57.3;
	//message.local_position.orientation.roll  = (atan2(2*q2*q3 + 2*q0*q1, 1-2*q1*q1-2*q2*q2))*57.3;
	current_yaw = atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1) + Pi;//North:0, south:Pi, East:Pi/2, West: Pi*3/2
//  ROS_INFO("current_yaw %f",current_yaw);
}
void chatterCallback_mode(const mavros::State &msg)
{
	if(msg.mode=="OFFBOARD") 
	{
		offboard_ready = true;
	}
	else 
	{
		offboard_ready = false;  
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
	}

}


void trajectory_Paras_generation_i(int num, float p0, float pf, float T, Matrix<float, 3, 3>& Paras_matrix)//num 0,1,2 reoresents  x, y, z
{
	float v0 = 0, a0 = 0, vf = 0, af = 0;
	MatrixXd delt_s(3,1);
	delt_s(0,0) = af-a0;
	delt_s(1,0) = vf-v0-a0*T;
	delt_s(2,0) = pf-p0-v0*T-0.5*a0*T*T;

	MatrixXd temp(3,3);
	temp << 60/pow(T,3),-360/pow(T,4),720/pow(T,5),-24/pow(T,2),168/pow(T,3),-360/pow(T,4),3/T,-24/pow(T,2),60/pow(T,3);
	//std::cout << temp;
	MatrixXd const_paras(3,1);//(alfa,beta,gamma)
	const_paras = temp * delt_s;
	//std::cout << const_Paras_matrix;
	Paras_matrix(num,0) = const_paras(0,0);
	Paras_matrix(num,1) = const_paras(1,0);
	Paras_matrix(num,2) = const_paras(2,0);
}

float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return 0.5*alfa*t*t+beta*t+gamma;
}

float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t, float p0)
{
	return alfa*pow(t,5)/120+beta*pow(t,4)/24+gamma*pow(t,3)/6+p0;
}
float p_simple_calculate(int num, float t, float vel, float p0)
{
	return p0 + vel * t;
}
float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return alfa*pow(t,4)/24+beta*pow(t,3)/6+gamma*t*t/2;
}

float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
	return alfa*pow(t,3)/6+beta*t*t/2+gamma*t;
}

int trapezoidalTraj(float start_pos, float ended_pos, 
	float MAX_v, float MAX_pitch_deg, float MAX_j,
	VectorXf& nodes_time, 
	VectorXf& nodes_vel, 
	VectorXf& nodes_pos, 
	float* p_max_acc)

{
	float MAX_a = 9.8*tan(MAX_pitch_deg/57.3f);
	float max_v, max_a, max_j = MAX_j;
	VectorXf nodes_t = VectorXf::Zero(8);
	VectorXf nodes_p = VectorXf::Zero(8);
	VectorXf nodes_v = VectorXf::Zero(8);
	VectorXf blocks_t = VectorXf::Zero(7);
	float duration_amax = MAX_v / MAX_a - MAX_a / MAX_j;
	float total_length = ended_pos - start_pos;
	float duration_vmax = (total_length - (1 / MAX_a * MAX_v * MAX_v + MAX_a / MAX_j * MAX_v)) / MAX_v;
	if(duration_amax<0 && duration_vmax>0){
		// ROS_INFO("MAX_a unreachable");
		max_a = sqrt(MAX_j * (MAX_v));
		max_v = MAX_v;
		blocks_t(1) = 0;
		duration_vmax = (total_length - (1 / max_a * max_v * max_v + max_a / MAX_j * max_v)) / max_v;
		if (duration_vmax<0){
				duration_vmax = 0;
			//   ROS_INFO("MAX_v unreachable");
			// ROS_INFO("Please use Muellers method");
			return 1;
		}
		blocks_t(3) = duration_vmax;
	}
	else if(duration_amax>0 && duration_vmax<0){
		// ROS_INFO("MAX_v unreachable");
		// ROS_INFO("Please use Muellers method");
		return 1;
	}
	else if(duration_amax<0 && duration_vmax<0){
		// ROS_INFO("Both MAX_a and MAX_v unreachable");
		// ROS_INFO("Please use Muellers method");
		return 1;
	}
	else{
		blocks_t(1) = duration_amax;
		blocks_t(3) = duration_vmax;
		max_a = MAX_a;
		max_v = MAX_v;
	}
	blocks_t(0) = max_a / max_j;
	blocks_t(2) = blocks_t(0);
	blocks_t(4) = blocks_t(2);
	blocks_t(5) = blocks_t(1);
	blocks_t(6) = blocks_t(0);
	nodes_t(0) = 0;
	for(int i = 1; i < 8; i++)
		nodes_t(i) = nodes_t(i - 1) + blocks_t(i - 1);
	nodes_v(0) = 0;
	nodes_v(1) = nodes_v(0) + max_j * blocks_t(0) * blocks_t(0) / 2;
	nodes_v(2) = nodes_v(1) + max_a * blocks_t(1);
	nodes_v(3) = nodes_v(2) + max_a * blocks_t(2) - max_j * blocks_t(2) * blocks_t(2) / 2;
	nodes_v(4) = nodes_v(3);
	nodes_v(5) = nodes_v(4) - max_j * blocks_t(4) * blocks_t(4) / 2;
	nodes_v(6) = nodes_v(5) - max_a * blocks_t(5);
	nodes_v(7) = 0;
	nodes_p(0) = start_pos;
	nodes_p(1) = nodes_p(0) + nodes_v(0) * blocks_t(0) + max_j * blocks_t(0) * blocks_t(0) * blocks_t(0)/6;
	nodes_p(2) = nodes_p(1) + nodes_v(1) * blocks_t(1) + max_a * blocks_t(1) * blocks_t(1) / 2;
	nodes_p(3) = nodes_p(2) + nodes_v(2) * blocks_t(2) + max_a * blocks_t(2) * blocks_t(2) / 2 - max_j * blocks_t(2) * blocks_t(2) * blocks_t(2) / 6;
	nodes_p(4) = nodes_p(3) + nodes_v(3) * blocks_t(3);
	nodes_p(5) = nodes_p(4) + nodes_v(4) * blocks_t(4) - max_j * blocks_t(4) * blocks_t(4) * blocks_t(4)/6;
	nodes_p(6) = nodes_p(5) + nodes_v(5) * blocks_t(5) - max_a * blocks_t(5) * blocks_t(5) / 2;
	nodes_p(7) = ended_pos;
	nodes_time = nodes_t;
	nodes_vel = nodes_v;
	nodes_pos = nodes_p;
	*p_max_acc = max_a;
	return 0;
}
float jerkPlan(float max_jerk, int stage)
{
	float jerk = 0;
	switch (stage){
	case 1:
		jerk = max_jerk;
	break;
	case 2:
		jerk = 0;
	break;
	case 3:
		jerk = -max_jerk;
	break;
	case 4:
		jerk = 0;
	break;
	case 5:
		jerk = -max_jerk;
	break;
	case 6:
		jerk = 0;
	break;
	case 7:
		jerk = max_jerk;
	break;
	default:
	break;
	}
	return jerk;
}
float accPlan(float max_jerk, float max_acc, float t, 
	int stage, const VectorXf& nodes_time)
{
	float tau = t - nodes_time(stage - 1);
	float acc = 0;
	switch (stage){
	case 1:
		acc = max_jerk * tau;
	break;
	case 2:
		acc = max_acc;
	break;
	case 3:
		acc = max_acc - max_jerk * tau;
	break;
	case 4:
		acc = 0;
	break;
	case 5:
		acc = -max_jerk * tau;
	break;
	case 6:
		acc = -max_acc;
	break;
	case 7:
		acc = -max_acc + max_jerk * tau;
	break;
	default:
	break;
	}
	return acc;
}
float velPlan(float max_jerk, float max_acc, float t, 
	int stage, const VectorXf& nodes_time, const VectorXf& nodes_vel)
{
	float tau = t - nodes_time(stage - 1);
	float vel = 0;
	switch (stage){
	case 1:
		vel = nodes_vel(0) + max_jerk * tau * tau / 2;
	break;
	case 2:
		vel = nodes_vel(1) + max_acc * tau;
	break;
	case 3:
		vel = nodes_vel(2) + max_acc * tau - max_jerk * tau * tau / 2;
	break;
	case 4:
		vel = nodes_vel(3);
	break;
	case 5:
		vel = nodes_vel(4) - max_jerk * tau * tau / 2;
	break;
	case 6:
		vel = nodes_vel(5) - max_acc * tau;
	break;
	case 7:
		vel = nodes_vel(6) - max_acc * tau + max_jerk * tau * tau / 2;
	break;
	default:
	break;
	}
	return vel;
}
float posPlan(float max_jerk, float max_acc, float t, 
	int stage, const VectorXf& nodes_time, 
	const VectorXf& nodes_vel, const VectorXf& nodes_pos)
{
	float tau = t - nodes_time(stage - 1);
	float pos = nodes_pos(7);
	switch (stage){
	case 1:
		pos = nodes_pos(0) + nodes_vel(0) * tau + max_jerk * tau * tau * tau / 6;
	break;
	case 2:
		pos = nodes_pos(1) + nodes_vel(1) * tau + max_acc * tau * tau / 2;
	break;
	case 3:
		pos = nodes_pos(2) + nodes_vel(2) * tau + max_acc * tau * tau / 2 - max_jerk * tau * tau * tau / 6;
	break;
	case 4:
		pos = nodes_pos(3) + nodes_vel(3) * tau;
	break;
	case 5:
		pos = nodes_pos(4) + nodes_vel(4) * tau - max_jerk * tau * tau * tau / 6;
	break;
	case 6:
		pos = nodes_pos(5) + nodes_vel(5) * tau - max_acc * tau * tau / 2;
	break;
	case 7:
		pos = nodes_pos(6) + nodes_vel(6) * tau - max_acc * tau * tau / 2 + max_jerk * tau * tau * tau / 6;
	break;
	default:
	break;
	}
	return pos;
}

void chatterCallback_extra_function(const mavros_extras::ExtraFunctionReceiver &msg)
{
	if(msg.laser_height_enable == 1) laser_fly_height_enable = true;
	else laser_fly_height_enable = false;
}


//Subscribe crop distance msg by CJ
void chatterCallback_crop_distance(const std_msgs::Float32 &msg)
{
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
