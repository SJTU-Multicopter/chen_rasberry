#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/OffboardRoutePointsConfirm.h>
#include <ros/console.h>

namespace mavplugin{

class OffboardRoutePointsConfirmPlugin : public MavRosPlugin{

public:
     OffboardRoutePointsConfirmPlugin():
     offboard_route_points_confirm_nh("~offboard_route_points_confirm"),
     uas(nullptr)
    { };

    void initialize(UAS &uas_){
        uas = &uas_;

        offboard_route_points_confirm_nh.param<std::string>("frame_id", frame_id, "offboard_route_points_confirm");
        //subcribe the topic and excute the callback function
        offboard_route_points_confirm_sub = offboard_route_points_confirm_nh.subscribe("/offboard_route_points_confirm",5,&OffboardRoutePointsConfirmPlugin::offboard_route_points_confirm_send_cb,this);
    }

    std::string get_name() {
        return "offboard_route_points_confirm";
    }


     const message_map get_rx_handlers() {
        return {

        };
    }


private:
    ros::NodeHandle offboard_route_points_confirm_nh;
    ros::Subscriber offboard_route_points_confirm_sub;
    UAS *uas;

    std::string frame_id;

    void offboard_route_points_confirm_send(int seq, int total, float px_1,float py_1,float ph_1,float px_2,float py_2,float ph_2){
        mavlink_message_t offboard_route_points_confirm_msg;

        mavlink_msg_offboard_setpoint_confirm_pack_chan(UAS_PACK_CHAN(uas),&offboard_route_points_confirm_msg, seq, total, px_1, py_1, ph_1, px_2, py_2, ph_2); //pack
        UAS_FCU(uas)->send_message(&offboard_route_points_confirm_msg); //send

        //ROS_INFO("size %d %f", offboard_route_points_confirm_msg.seq,a);

    }

    //callbacks
    void offboard_route_points_confirm_send_cb(const mavros_extras::OffboardRoutePointsConfirm &msg){
        offboard_route_points_confirm_send(msg.seq, msg.total, msg.px_1, msg.py_1, msg.ph_1, msg.px_2, msg.py_2, msg.ph_2);
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::OffboardRoutePointsConfirmPlugin, mavplugin::MavRosPlugin)
