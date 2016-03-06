#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/OffboardRoutePoints.h>
#include "std_msgs/Float32.h"
#include <ros/console.h>

namespace mavplugin{

class OffboardRoutePointsPlugin : public MavRosPlugin{

public:
     OffboardRoutePointsPlugin():
     offboard_route_points_nh("~offboard_route_points"),
     uas(nullptr)
    { };

    void initialize(UAS &uas_){
        uas = &uas_;

        offboard_route_points_nh.param<std::string>("frame_id", frame_id, "offboard_route_points");
        //subcribe the topic and excute the callback function
        offboard_route_points_sub = offboard_route_points_nh.subscribe("/offboard_route_points",5,&OffboardRoutePointsPlugin::offboard_route_points_send_cb,this);

    }

    std::string get_name() {
        return "offboard_route_points";
    }


     const message_map get_rx_handlers() {
        return {

        };
    }


private:
    ros::NodeHandle offboard_route_points_nh;
    ros::Subscriber offboard_route_points_sub;
    UAS *uas;

    std::string frame_id;
    void offboard_route_points_send(float px_1,float py_1,float ph_1,float px_2,float py_2,float ph_2, float yaw, int seq, int total){
        mavlink_message_t offboard_route_points_msg;

        mavlink_msg_offboard_setpoint_pack_chan(UAS_PACK_CHAN(uas), &offboard_route_points_msg, px_1, py_1, ph_1, px_2, py_2, ph_2, yaw, seq, total); //pack
        UAS_FCU(uas)->send_message(&offboard_route_points_msg); //send 

        //ROS_INFO("size %d %f", offboard_route_points_msg.seq, a);

    }

    //callbacks
    void offboard_route_points_send_cb(const mavros_extras::OffboardRoutePoints &msg){
        offboard_route_points_send(msg.px_1, msg.py_1, msg.ph_1, msg.px_2, msg.py_2, msg.ph_2, msg.yaw, msg.seq, msg.total);
        //ROS_INFO("%f", msg.seq);

    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::OffboardRoutePointsPlugin, mavplugin::MavRosPlugin)
