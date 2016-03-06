#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/FieldSize.h>
#include "std_msgs/Float32.h"
#include <ros/console.h>

namespace mavplugin{

class FieldSizePlugin : public MavRosPlugin{

public:
     FieldSizePlugin():
     field_size_nh("~field_size"),
     uas(nullptr)
    { };

    void initialize(UAS &uas_){
        uas = &uas_;

        field_size_nh.param<std::string>("frame_id", frame_id, "field_size");
        //subcribe the topic and excute the callback function
        field_size_sub = field_size_nh.subscribe("/field_size_set",5,&FieldSizePlugin::field_size_send_cb,this);

    }

    std::string get_name() {
        return "field_size";
    }


     const message_map get_rx_handlers() {
        return {

        };
    }


private:
    ros::NodeHandle field_size_nh;
    ros::Subscriber field_size_sub;
    UAS *uas;

    std::string frame_id;

    void field_size_send(float a, float b, float c, uint d){
        mavlink_message_t field_size_msg;

        mavlink_msg_field_size_pack_chan(UAS_PACK_CHAN(uas),&field_size_msg,a,b,c,d); //pack
        UAS_FCU(uas)->send_message(&field_size_msg); //send

        //ROS_INFO("size %d %f", field_size_msg.seq, a);

    }

    //callbacks
    void field_size_send_cb(const mavros_extras::FieldSize &msg){
        field_size_send(msg.length, msg.width, msg.height, msg.times);
        ROS_INFO("%f", msg.length);


    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::FieldSizePlugin, mavplugin::MavRosPlugin)
