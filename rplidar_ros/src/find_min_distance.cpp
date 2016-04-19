#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "mavros_extras/LaserDistance.h"
#include <math.h>

mavros_extras::LaserDistance pos;

void scanCallback(const sensor_msgs::LaserScan laser)
{
	float min_distance;
	int angle;

	min_distance = 6.0;
	angle = 0;
	for(int i=0; i<laser.ranges.size(); i=i+1)
	{
		if(laser.intensities[i] > 12.0)
		{
			if (laser.ranges[i] > 0.9 && laser.ranges[i] < laser.range_max){
				if(laser.ranges[i] < min_distance)
				{
					if(laser.intensities[i] > 18.0)
					{
						min_distance = laser.ranges[i];
						angle = i;
					}
					if(i == 0){
						if(fabs(laser.ranges[i] - laser.ranges[i + 1]) < 0.1 || fabs(laser.ranges[i] - laser.ranges[laser.ranges.size() - 1]) < 0.1){
							if(laser.intensities[i] + laser.intensities[i + 1] + laser.intensities[laser.ranges.size() - 1] > 39.0)
							{
								min_distance = laser.ranges[i];
								angle = i;
							}
						}
					}else if(i < laser.ranges.size() - 1){
						if(fabs(laser.ranges[i] - laser.ranges[i + 1]) < 0.1 || fabs(laser.ranges[i] - laser.ranges[i - 1]) < 0.1){
							if(laser.intensities[i] + laser.intensities[i + 1] + laser.intensities[i - 1] > 39.0)
							{
								min_distance = laser.ranges[i];
								angle = i;
							}
						}
					}else{
						if(fabs(laser.ranges[i] - laser.ranges[i - 1]) < 0.1 || fabs(laser.ranges[i] - laser.ranges[0]) < 0.1){
							if(laser.intensities[i] + laser.intensities[i - 1] + laser.intensities[0] > 39.0)
							{
								min_distance = laser.ranges[i];
								angle = i;
							}
						}
					}	 
				}
			}
		}
	}
	pos.min_distance = min_distance*100;
	pos.angle = angle;
} 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_min_distance");
	ros::NodeHandle n;
	ros::Subscriber scan_sub = n.subscribe("/scan_horizontal", 1, scanCallback);
	ros::Publisher pub = n.advertise<mavros_extras::LaserDistance>("/laser_send", 5);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		pub.publish(pos);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
