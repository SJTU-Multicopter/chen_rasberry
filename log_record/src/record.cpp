#include "ros/ros.h"  
#include "geometry_msgs/PoseStamped.h" 
#include <mavros/State.h>
#include <mavros/SetPointLocal.h>
#include <mavros/Vector3.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <math.h>  
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <time.h> 
#include <stdio.h> 
#include <mavros_extras/PositionSetpoint.h>
#include <mavros_extras/PositionSetpoint.h>

using Eigen::MatrixXd;

MatrixXd St_matrix(3,3);//(p,v,a)
int mode = 0;
bool flag_p = false;
bool flag_v = false;
bool flag_a = false;
float time_stamp = 0.0;
float yaw_read=0.0;

void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg);
void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg);
void chatterCallback_receive_setpoint_local(const mavros_extras::PositionSetpoint &setpoint);
void chatterCallback_Mode(const mavros::State &msg);

int main(int argc, char **argv)  
{
	ros::init(argc, argv, "record");

	ros::NodeHandle nh; 
	ros::Subscriber localposition_sub = nh.subscribe("/mavros/local_position/local", 500,chatterCallback_local_position);
    ros::Subscriber setpoint_sub = nh.subscribe("/offboard/setpoints_local", 5, chatterCallback_receive_setpoint_local);
    ros::Subscriber setpoint_sub2 = nh.subscribe("/offboard/setpoints_raw", 2, chatterCallback_receive_setpoint_raw);
    ros::Subscriber mode_sub = nh.subscribe("/mavros/state", 100,chatterCallback_Mode);
    //imitate data
    //ros::Subscriber localposition_sub = nh.subscribe("/offboard/position_imitate", 500,chatterCallback_local_position);
    //ros::Subscriber imu_sub = nh.subscribe("/offboard/acceleration_imitate", 500,chatterCallback_imu_data);
    //ros::Subscriber velocity_sub = nh.subscribe("/offboard/velocity_imitate", 500,chatterCallback_local_velocity);
    
    St_matrix<<0.0,0.0,0.0,
               0.0,0.0,0.0,
               0.0,0.0,0.0;
    time_t tt = time(NULL);
    tm* t= localtime(&tt);
    char name[20];
    sprintf(name,"%d-%02d-%02d-%02d:%02d:%02d.txt",
      t->tm_year + 1900,
      t->tm_mon + 1,
      t->tm_mday,
      t->tm_hour,
      t->tm_min,
      t->tm_sec);

    char path[50]="/home/ubuntu/catkin_ws/log/";
    strcat(path,name);
    std::cout<<"file saved in "<<path;
    FILE *pTxtFile = NULL;

    pTxtFile = fopen(path, "w+");
    if (pTxtFile == NULL)
    {
        printf("Open file failed! The program exist!\n");
        return 0;
    }
    
    std::cout<<"writing...\n";
    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(flag_p){
           fprintf(pTxtFile,"%f M%d P%f %f %f SPR%f %f %f SPL%f %f %f Y%f\n", time_stamp, mode,
             St_matrix(0,0),St_matrix(1,0),St_matrix(2,0),St_matrix(0,1),St_matrix(1,1),St_matrix(2,1),St_matrix(0,2),St_matrix(1,2),St_matrix(2,2),yaw_read);
           time_stamp += 0.1;
        }

    	ros::spinOnce();  
    	loop_rate.sleep();
    }
    
    fclose(pTxtFile);
	return 0;
}

void chatterCallback_local_position(const geometry_msgs::PoseStamped &msg)
{
	St_matrix(0,0) = msg.pose.position.x;
	St_matrix(1,0) = msg.pose.position.y;
	St_matrix(2,0) = msg.pose.position.z;
	flag_p = true;
}


void chatterCallback_Mode(const mavros::State &msg)//模式
{
    if(msg.mode=="MANUAL") mode=1;
    else if(msg.mode=="OFFBOARD") mode=7;
    else if(msg.mode=="POSCTR") mode=3;
    else mode = 5;
}

void chatterCallback_receive_setpoint_raw(const mavros_extras::PositionSetpoint &msg)
{
  St_matrix(0,1) = msg.px;
  St_matrix(1,1) = msg.py;
  St_matrix(2,1) = msg.ph;
  yaw_read = msg.yaw;
}

void chatterCallback_receive_setpoint_local(const mavros_extras::PositionSetpoint &setpoint)
{
  St_matrix(0,2) = setpoint.px;
  St_matrix(1,2)= setpoint.py;
  St_matrix(2,2) = setpoint.ph;
}
