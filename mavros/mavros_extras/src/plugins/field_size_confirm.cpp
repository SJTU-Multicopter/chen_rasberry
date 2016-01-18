#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/FieldSizeConfirm.h>
#include <ros/console.h>

namespace mavplugin{

class FieldSizeConfirmPlugin : public MavRosPlugin{

public:
     FieldSizeConfirmPlugin():
     field_size_confirm_nh("~field_size_confirm"),
     uas(nullptr)
    { };

    void initialize(UAS &uas_){
        uas = &uas_;

        field_size_confirm_nh.param<std::string>("frame_id", frame_id, "field_size_confirm");
        //subcribe the topic and excute the callback function
        field_size_confirm_sub = field_size_confirm_nh.subscribe("/field_size_confirm",500,&FieldSizeConfirmPlugin::field_size_confirm_send_cb,this);
    }

    std::string get_name() {
        return "field_size_confirm";
    }


     const message_map get_rx_handlers() {
        return {

        };
    }


private:
    ros::NodeHandle field_size_confirm_nh;
    ros::Subscriber field_size_confirm_sub;
    UAS *uas;

    std::string frame_id;

    void field_size_confirm_send(float a, float b, float c, uint d, uint e){
        mavlink_message_t field_size_confirm_msg;

        mavlink_msg_field_size_confirm_pack_chan(UAS_PACK_CHAN(uas),&field_size_confirm_msg,a,b,c,d,e); //pack
        UAS_FCU(uas)->send_message(&field_size_confirm_msg); //send

        //ROS_INFO("size %d %f", field_size_confirm_msg.seq,a);

    }

    //callbacks
    void field_size_confirm_send_cb(const mavros_extras::FieldSizeConfirm &msg){
        field_size_confirm_send(msg.length, msg.width, msg.height, msg.times, msg.confirm);
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::FieldSizeConfirmPlugin, mavplugin::MavRosPlugin)
