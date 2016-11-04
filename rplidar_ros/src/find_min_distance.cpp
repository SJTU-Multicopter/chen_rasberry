#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "mavros_extras/LaserDistance.h"
#include <math.h>

class ScanProcess
{
public: 
  ScanProcess();
private:
  ros::NodeHandle n;
  ros::Subscriber scan_sub;
  ros::Publisher pub;
  mavros_extras::LaserDistance pos;
  
  void scanCallback(const sensor_msgs::LaserScan laser);
};

ScanProcess::ScanProcess()
{
  scan_sub = n.subscribe("/scan_horizontal", 1, &ScanProcess::scanCallback, this);
  pub = n.advertise<mavros_extras::LaserDistance>("/laser_send", 5);
}

void ScanProcess::scanCallback(const sensor_msgs::LaserScan laser)
{
  float min_distance;
  int angle;

  min_distance = laser.range_max;
  angle = 0;
  for(int i=0; i<laser.ranges.size(); i=i+1)
  {
    if (laser.ranges[i] > 0.9 && laser.ranges[i] < laser.range_max)
    {
      if(laser.ranges[i] < min_distance)
      {
        if(i == 0)
        {
          if(fabs(laser.ranges[i] - laser.ranges[i + 1]) < 0.5)
          {
            min_distance = laser.ranges[i];
            angle = i;
          }
        }else if(i < laser.ranges.size() - 1)
        {
          if(fabs(laser.ranges[i] - laser.ranges[i + 1]) < 0.5 || fabs(laser.ranges[i] - laser.ranges[i - 1]) < 0.5)
          {
            min_distance = laser.ranges[i];
            angle = i;
          }
        }else
        {
          if(fabs(laser.ranges[i] - laser.ranges[i - 1]) < 0.5)
          {
            min_distance = laser.ranges[i];
            angle = i;
          }
        } 
      }   
    }
  }
  pos.min_distance = min_distance;
  pos.angle = angle*0.25 - 45;
  if(pos.angle < 0) pos.angle = pos.angle + 360;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_min_distance");
  ScanProcess ScanProcess;
  ros::spin();
}
