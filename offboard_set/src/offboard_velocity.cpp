#include "ros/ros.h"  
#include "geometry_msgs/TwistStamped.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>
#include "mavros/CommandBool.h"
#include "mavros/SetMode.h"
#include <sstream>  
#include <Eigen/Dense>
#include <math.h>  
#include <iostream>

#define LOOP_RATE 20

using Eigen::MatrixXd;

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_imu_data(const sensor_msgs::Imu &msg);
void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg);

void init_velocity(); 
void set_velocity(float x, float y, float z, float yr);
void trajectory_Paras_generation_i(int num, float p0, float v0, float a0, float pf, float vf, float af, float T);
void trajectory_generation(float T,float pxf, float pyf, float pzf,float vxf, float vyf, float vzf, float axf, float ayf, float azf);
float j_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t);
float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t);



geometry_msgs::TwistStamped msg;
MatrixXd S0_matrix(3,3);//(p,v,a)
MatrixXd St_matrix(3,3);//(p,v,a)
MatrixXd Paras_matrix(3,3);//(alfa,beta,gamma)
MatrixXd St_optimal_matrix(3,3);//(p,v,a)
MatrixXd j_optimal_matrix(1,3);//(jx,jy,jz)

float t = 0.0;

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "offboard_velocity"); 

  init_velocity();
  S0_matrix<<0.0,0.0,0.0,
               0.0,0.0,0.0,
               0.0,0.0,0.0;
  St_matrix<<0.0,0.0,0.0,
               0.0,0.0,0.0,
               0.0,0.0,0.0;
  ros::NodeHandle nh;

  ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 500,chatterCallback_local_position);
  ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1000,chatterCallback_imu_data);
  ros::Subscriber velocity_sub = nh.subscribe("/mavros/local_position/local_velocity", 1000,chatterCallback_local_velocity);

  ros::Publisher offboard_pub_v = nh.advertise<geometry_msgs::TwistStamped>("offboard/velocity", 1000);  
  
  ros::Rate loop_rate(LOOP_RATE);
  
  while(ros::ok()&&t<=10.0){
    
    msg.header.stamp = ros::Time::now();
    trajectory_generation(30.0, 20.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    ROS_INFO("%f %f %f",  msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z); 
    offboard_pub_v.publish(msg); 

    t += 1.0/(float)LOOP_RATE;
    ros::spinOnce();
    loop_rate.sleep();
  } 
  
  
  return 0;  
}  

void init_velocity()
{
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;
}

void set_velocity(float x, float y, float z, float yr=0.0)
{
    msg.twist.linear.x = x*100;
    msg.twist.linear.y = y*100;
    msg.twist.linear.z = z;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = yr;
}

void trajectory_generation(float T,float pxf, float pyf, float pzf,float vxf=0.0, float vyf=0.0, float vzf=0.0, float axf=0.0, float ayf=0.0, float azf=0.0)
{
  //S0_matrix = St_matrix; 

  trajectory_Paras_generation_i(0, S0_matrix(0,0),S0_matrix(0,1),S0_matrix(0,2),pxf,vxf,axf,T);
  trajectory_Paras_generation_i(1, S0_matrix(1,0),S0_matrix(1,1),S0_matrix(1,2),pyf,vyf,ayf,T);
  trajectory_Paras_generation_i(2, S0_matrix(2,0),S0_matrix(2,1),S0_matrix(2,2),pzf,vzf,azf,T);

  j_optimal_matrix(0,0) = j_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t); //x
  j_optimal_matrix(0,1) = j_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t); //y
  j_optimal_matrix(0,2) = j_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t); //z
  //x
  St_optimal_matrix(0,0) = p_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t); 
  St_optimal_matrix(0,1) = v_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t);
  St_optimal_matrix(0,2) = a_optimal_calculate(0,Paras_matrix(0,0),Paras_matrix(0,1),Paras_matrix(0,2),t);

  //y
  St_optimal_matrix(1,0) = p_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t); 
  St_optimal_matrix(1,1) = v_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t);
  St_optimal_matrix(1,2) = a_optimal_calculate(1,Paras_matrix(1,0),Paras_matrix(1,1),Paras_matrix(1,2),t);

  //z
  St_optimal_matrix(2,0) = p_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t); 
  St_optimal_matrix(2,1) = v_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t);
  St_optimal_matrix(2,2) = a_optimal_calculate(2,Paras_matrix(2,0),Paras_matrix(2,1),Paras_matrix(2,2),t);

  set_velocity(St_optimal_matrix(0,1),St_optimal_matrix(1,1),St_optimal_matrix(2,1));

}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
  St_matrix(0,0) = msg.pose.position.x;
  St_matrix(1,0) = msg.pose.position.y;
  St_matrix(2,0) = msg.pose.position.z;
}

void chatterCallback_imu_data(const sensor_msgs::Imu &msg)
{
  St_matrix(0,2) = msg.linear_acceleration.x;
  St_matrix(1,2) = msg.linear_acceleration.y;
  St_matrix(2,2) = msg.linear_acceleration.z;
}
void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg)
{
  St_matrix(0,1) = msg.x;
  St_matrix(1,1) = msg.y;
  St_matrix(2,1) = msg.z;
}

void trajectory_Paras_generation_i(int num, float p0, float v0, float a0, float pf, float vf, float af, float T)//num 0,1,2 reoresents  x, y, z
{
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

float p_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
  return alfa*pow(t,5)/120+beta*pow(t,4)/24+gamma*pow(t,3)/6+S0_matrix(num,2)*t*t/2+S0_matrix(num,1)*t+S0_matrix(num,0);
}

float v_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
  return alfa*pow(t,4)/24+beta*pow(t,3)/6+gamma*t*t/2+S0_matrix(num,2)*t+S0_matrix(num,1);
}

float a_optimal_calculate(int num, float alfa, float beta, float gamma, float t)
{
  return alfa*pow(t,3)/6+beta*t*t/2+gamma*t+S0_matrix(num,2);
}