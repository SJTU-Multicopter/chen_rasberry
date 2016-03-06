#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/PumpController.h>
#include <ros/console.h>

namespace mavplugin{

class PumpControllerPlugin : public MavRosPlugin{

public:
     PumpControllerPlugin():
     pump_controller_nh("~pump_controller"),
     uas(nullptr)
    { };

    void initialize(UAS &uas_){
        uas = &uas_;

        pump_controller_nh.param<std::string>("frame_id", frame_id, "pump_controller");
        //subcribe the topic and excute the callback function
        pump_controller_sub = pump_controller_nh.subscribe("/pump_controller",2,&PumpControllerPlugin::pump_controller_send_cb,this);

    }

    std::string get_name() {
        return "pump_controller";
    }


     const message_map get_rx_handlers() {
        return {

        };
    }


private:
    ros::NodeHandle pump_controller_nh;
    ros::Subscriber pump_controller_sub;
    UAS *uas;

    std::string frame_id;

    void pump_controller_send(float a, float b){
        mavlink_message_t pump_controller_msg;

        mavlink_msg_pump_controller_pack_chan(UAS_PACK_CHAN(uas),&pump_controller_msg,a,b); //pack
        UAS_FCU(uas)->send_message(&pump_controller_msg); //send

        //ROS_INFO("size %d %f", field_size_msg.seq, a);

    }

    //callbacks
    void pump_controller_send_cb(const mavros_extras::PumpController &msg){
        pump_controller_send(msg.pump_speed_sp, msg.spray_speed_sp);

    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::PumpControllerPlugin, mavplugin::MavRosPlugin)
