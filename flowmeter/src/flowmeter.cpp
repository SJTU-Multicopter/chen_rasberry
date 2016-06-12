#include <wiringPi.h>

#include <std_msgs/String.h>
#include <flowmeter/flowrate.h>
#include "ros/ros.h"
int second_cnt = 0;
#define FLOW_SIGNAL_PIN 15
void flow_interrupt(void)
{
	int status = digitalRead(FLOW_SIGNAL_PIN);
	if (status == 1) {//up edge
		second_cnt++;
	} else { //down edge

	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "flowmeter");
	ros::NodeHandle n;
	wiringPiSetup ();
//	pinMode(USB_OUT_PIN, OUTPUT);
	pinMode(FLOW_SIGNAL_PIN, INPUT);
	wiringPiISR (FLOW_SIGNAL_PIN, INT_EDGE_BOTH, &flow_interrupt) ;
	ros::Publisher flowrate_pub = n.advertise<flowmeter::flowrate>("flowrate",5);
	ros::Rate loop_rate(1);
	while (ros::ok()){
		int freq = second_cnt;
		float flowrate = freq / 48.0;
		ROS_INFO("flowrate: %f\n", flowrate);
		flowmeter::flowrate msg;
		msg.flowrate = flowrate;
		flowrate_pub.publish(msg);
		second_cnt = 0;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
