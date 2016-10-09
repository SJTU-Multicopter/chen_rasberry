#include <wiringPi.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include <flowmeter/flowrate.h>
#include "mavros_extras/ExtraFunctionReceiver.h"

int second_cnt = 0;
float fuel_left = 0;
int but_hold_time = 0;
//std_msgs::Float32 msg;
flowmeter::flowrate flow_msg;
#define FLOW_SIGNAL_PIN 15
#define POWER_SRC 4
#define SHUT_BUT_PIN 5
void flow_interrupt(void)
{
	int status = digitalRead(FLOW_SIGNAL_PIN);
	if (status == 1) {//up edge
		second_cnt++;
	} else { //down edge

	}
}
void resetCallback(const mavros_extras::ExtraFunctionReceiver &msg)
{
	int msg_add_one = msg.add_one;
	
	static int last_data = 0;
	if(msg_add_one / 100 != last_data)
		fuel_left = (msg_add_one % 100) / 10.0;
	last_data = msg_add_one/100;

}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "flowmeter");
	ros::NodeHandle n;
	wiringPiSetup ();
//	pinMode(USB_OUT_PIN, OUTPUT);
	pinMode(FLOW_SIGNAL_PIN, INPUT);
	// pinMode(POWER_SRC, OUTPUT);
	// pinMode(SHUT_BUT_PIN, INPUT);
	// digitalWrite(POWER_SRC, HIGH);
	wiringPiISR (FLOW_SIGNAL_PIN, INT_EDGE_BOTH, &flow_interrupt) ;
    ros::Publisher flowrate_pub = n.advertise<flowmeter::flowrate>("flowrate",5);
	ros::Subscriber reset_sub = n.subscribe("/mavros/extra_function_receiver/extra_function_receiver",5,resetCallback);
	ros::Rate loop_rate(1);
	while (ros::ok()){
		// if(digitalRead(SHUT_BUT_PIN))
		// 	but_hold_time++;
		// else
		// 	but_hold_time = 0;
		// if(but_hold_time > 3){
		// 	fuel_left = 0;
		// 	but_hold_time = 0;
		// }

		int freq = second_cnt;
		float flowrate = freq / 48.0;
		fuel_left -= flowrate/60.0;
		if(fuel_left<0)
			fuel_left=0;
		ROS_INFO("flowrate: %f\n", flowrate);

        flow_msg.flowrate = flowrate;
        flow_msg.fuel_left = fuel_left;
		flowrate_pub.publish(flow_msg);
		second_cnt = 0;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
