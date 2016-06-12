#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/LaserDistance.h>
#include "std_msgs/Float32.h" 
#include <ros/console.h>

namespace mavplugin{

class LaserDistancePlugin : public MavRosPlugin{

public:
	 LaserDistancePlugin():
	 laser_distance_nh("~laser_distance"),
	 uas(nullptr)
    { };

    void initialize(UAS &uas_){
    	uas = &uas_;
        
    	laser_distance_nh.param<std::string>("frame_id", frame_id, "laser_distance");
        //subcribe the topic and excute the callback function
    	laser_distance_sub = laser_distance_nh.subscribe("/laser_send",5,&LaserDistancePlugin::laser_distance_send_cb,this);
        crop_height_sub = laser_distance_nh.subscribe("/crop_dist",5,&LaserDistancePlugin::crop_distance_send_cb,this);
        flowrate_sub = laser_distance_nh.subscribe("/flowrate",5,&LaserDistancePlugin::flowrate_send_cb,this);
    }
    
    std::string get_name() {
		return "laser_distance";
	}


     const message_map get_rx_handlers() {
		return {
			     
		};
	}


private:
	ros::NodeHandle laser_distance_nh;
	ros::Subscriber laser_distance_sub;
    ros::Subscriber crop_height_sub;
    ros::Subscriber flowrate_sub;
	UAS *uas;

    float crop_dist;
    float flowrate;

	std::string frame_id;

    void laser_distance_send(float a, float b, float c, float d){
    	mavlink_message_t laser_distance_msg;

    	mavlink_msg_laser_distance_pack_chan(UAS_PACK_CHAN(uas),&laser_distance_msg,a,b,c,d); //pack
    	UAS_FCU(uas)->send_message(&laser_distance_msg); //send
    	
    }
    
    //callbacks
    void laser_distance_send_cb(const mavros_extras::LaserDistance &msg){
        laser_distance_send(msg.min_distance,msg.angle,crop_dist,flowrate);
    }

    void crop_distance_send_cb(const std_msgs::Float32 &msg){
        crop_dist = msg.data;
    }
    void flowrate_send_cb(const std_msgs::Float32 &msg){
        flowrate = msg.data;
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::LaserDistancePlugin, mavplugin::MavRosPlugin)
