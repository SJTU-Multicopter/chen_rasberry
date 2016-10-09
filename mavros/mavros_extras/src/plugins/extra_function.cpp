#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/ExtraFunction.h>
#include "std_msgs/Float32.h"
#include <ros/console.h>

namespace mavplugin{

class ExtraFunctionPlugin : public MavRosPlugin{

public:
     ExtraFunctionPlugin():
     extra_function_nh("~extra_function"),
     uas(nullptr)
    { };

    void initialize(UAS &uas_){
        uas = &uas_;

        extra_function_nh.param<std::string>("frame_id", frame_id, "extra_function");
        //subcribe the topic and excute the callback function
        extra_function_sub = extra_function_nh.subscribe("/extra_function",1,&ExtraFunctionPlugin::extra_function_send_cb,this);

    }

    std::string get_name() {
        return "extra_function";
    }


     const message_map get_rx_handlers() {
        return {

        };
    }


private:
    ros::NodeHandle extra_function_nh;
    ros::Subscriber extra_function_sub;
    UAS *uas;

    std::string frame_id;
    void extra_function_send(unsigned short obs_avoid_enable, unsigned short laser_height_enable, unsigned short add_one, unsigned short add_two, unsigned short add_three){
        mavlink_message_t extra_function_msg;

        mavlink_msg_extra_function_pack_chan(UAS_PACK_CHAN(uas), &extra_function_msg,obs_avoid_enable, laser_height_enable, add_one, add_two, add_three); //pack
        UAS_FCU(uas)->send_message(&extra_function_msg); //send 

        //ROS_INFO("size %d %f", extra_function_msg.seq, a);

    }

    //callbacks
    void extra_function_send_cb(const mavros_extras::ExtraFunction &msg){
        extra_function_send(msg.obs_avoid_enable, msg.laser_height_enable, msg.add_one, msg.add_two, msg.add_three);
        //ROS_INFO("%f", msg.seq);

    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::ExtraFunctionPlugin, mavplugin::MavRosPlugin)
