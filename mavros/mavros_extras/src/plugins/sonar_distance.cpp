#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/SonarDistance.h>
#include <flowmeter/flowrate.h>
#include "std_msgs/Float32.h" 
#include <ros/console.h>

namespace mavplugin{

class SonarDistancePlugin : public MavRosPlugin{

public:
	 SonarDistancePlugin():
	 sonar_distance_nh("~sonar_distance"),
	 uas(nullptr)
    { };

    void initialize(UAS &uas_){
    	uas = &uas_;
        
    	sonar_distance_nh.param<std::string>("frame_id", frame_id, "sonar_distance");
        //subcribe the topic and excute the callback function
    	sonar_distance_sub = sonar_distance_nh.subscribe("/flowmeter",5,&SonarDistancePlugin::sonar_distance_send_cb,this);

    }
    
    std::string get_name() {
		return "sonar_distance";
	}


     const message_map get_rx_handlers() {
		return {
			     //MESSAGE_HANDLER(212, &SonarDistancePlugin::sonar_distance_send_cb)
		};
	}


private:
	ros::NodeHandle sonar_distance_nh;
	ros::Subscriber sonar_distance_sub;
	UAS *uas;

	std::string frame_id;

    void sonar_distance_send(float a, float b, float c, float d, float e, float f, float g){
    	mavlink_message_t sonar_distance_msg;

    	mavlink_msg_sonar_distance_pack_chan(UAS_PACK_CHAN(uas),&sonar_distance_msg,a,b,c,d,e,f,g); //pack
    	UAS_FCU(uas)->send_message(&sonar_distance_msg); //send
        //mavlink_msg_sonar_distance_send(MAVLINK_COMM_1,a,b,c,d,e,f);
        ROS_INFO("float_a %d %d", sonar_distance_msg.seq,sonar_distance_msg.len);
    	//uas->mav_link->send_message(&msg);
    }
    
    //callbacks
    void sonar_distance_send_cb(const flowmeter::flowrate &msg){
        sonar_distance_send(msg.flowrate,msg.fuel_left,0.0,0.1,0.2,0.3,0.4);
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::SonarDistancePlugin, mavplugin::MavRosPlugin)
