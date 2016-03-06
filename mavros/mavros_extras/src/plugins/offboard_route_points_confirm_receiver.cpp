#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/OffboardRoutePointsConfirm.h>


//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class OffboardRoutePointsReceiverConfirmPlugin : public MavRosPlugin{

public:
        OffboardRoutePointsReceiverConfirmPlugin():
                offboard_route_points_receiver_confirm_nh("~offboard_route_points_receiver_confirm"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		offboard_route_points_receiver_confirm_nh.param<std::string>("frame_id", frame_id, "offboard_route_points_receiver_confirm");
                    offboard_route_points_receiver_confirm_pub = offboard_route_points_receiver_confirm_nh.advertise<mavros_extras::OffboardRoutePointsConfirm>("offboard_route_points_receiver_confirm", 5); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_OFFBOARD_SETPOINT_CONFIRM, &OffboardRoutePointsReceiverConfirmPlugin::handle_offboard_route_points)
		};
	}

private:
        ros::NodeHandle offboard_route_points_receiver_confirm_nh;
        ros::Publisher offboard_route_points_receiver_confirm_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_offboard_route_points(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
                mavlink_offboard_setpoint_confirm_t setpoint_msg;
                mavlink_msg_offboard_setpoint_confirm_decode(msg, &setpoint_msg);//decode 
                
               
                auto setpoint_mav_msg = boost::make_shared<mavros_extras::OffboardRoutePointsConfirm>();//define a msg 
                setpoint_mav_msg->header.stamp = ros::Time::now();

                setpoint_mav_msg->px_1 = setpoint_msg.px_1; 
                setpoint_mav_msg->py_1 = setpoint_msg.py_1; 
                setpoint_mav_msg->ph_1 = setpoint_msg.ph_1; 
                setpoint_mav_msg->px_2 = setpoint_msg.px_2;
                setpoint_mav_msg->py_2 = setpoint_msg.py_2;
                setpoint_mav_msg->ph_2 = setpoint_msg.ph_2;
                setpoint_mav_msg->seq = setpoint_msg.seq;
                setpoint_mav_msg->total = setpoint_msg.total;
                
                offboard_route_points_receiver_confirm_pub.publish(setpoint_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::OffboardRoutePointsReceiverConfirmPlugin, mavplugin::MavRosPlugin)
