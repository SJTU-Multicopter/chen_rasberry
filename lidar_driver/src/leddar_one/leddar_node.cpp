#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "leddar_one/LeddarOne.h"

#include "ros/ros.h"
#include "lidar_driver/Lidar.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "leddar_node");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<lidar_driver::Lidar>("/lidar", 1000);
	ros::Rate loop_rate(20);

	char sPortName[LT_MAX_PORT_NAME_LEN] = "Lidar";
	unsigned int uBaudRate = 115200;
	unsigned int uParity = LT_PARITY_NONE;
	unsigned int uStopBits = 1;
	int   lAddress = 1;
	void* hConnection;
	LtResult result;
	LtAcquisition lAcquisition;
	lidar_driver::Lidar msg;

	result = LeddarOneConnect(&hConnection, sPortName, lAddress, uBaudRate, uParity, uStopBits);
	if (result == LT_SUCCESS)
	{
		ROS_INFO("Connect successfully!");
		while(ros::ok())
		{
			if(LeddarOneGetResults(hConnection, &lAcquisition) == LT_SUCCESS)
			{
				LtDetection *lDetections = lAcquisition.mDetections;
				for (int i = 0; i < lAcquisition.mDetectionCount; ++i)
				{
					msg.header.stamp = ros::Time::now();
					msg.distance.data = lDetections[i].mDistance;
					msg.amplitude.data = lDetections[i].mAmplitude;
					msg.temperature.data = lAcquisition.mTemperature;
					ROS_INFO("distance:%f",lDetections[i].mDistance);
					pub.publish(msg);
				}
			}else
			{
				ROS_INFO("Get result error!");
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}else{
		printf("Connection failed (error %i)!\n", (int)result);
	}

	return 0;
}
