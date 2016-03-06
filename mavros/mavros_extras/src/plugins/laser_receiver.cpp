#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/LaserDistance.h>
#include "std_msgs/Float32.h" 

//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class LaserReceiverPlugin : public MavRosPlugin{

public:
        LaserReceiverPlugin():
                laser_receiver_nh("~laser_receiver"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		laser_receiver_nh.param<std::string>("frame_id", frame_id, "laser_receiver");
        laser_receiver_pub = laser_receiver_nh.advertise<mavros_extras::LaserDistance>("laser_receiver", 5); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_LASER_DISTANCE, &LaserReceiverPlugin::handle_laser_distance)
		};
	}

private:
        ros::NodeHandle laser_receiver_nh;
        ros::Publisher laser_receiver_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_laser_distance(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_laser_distance_t laser_msg;
		mavlink_msg_laser_distance_decode(msg, &laser_msg);//decode 
                
               
                auto laser_mav_msg = boost::make_shared<mavros_extras::LaserDistance>();//define a msg the same type as  mavros_extras::ClarenceNewMavros
                laser_mav_msg->header.stamp = ros::Time::now();

                laser_mav_msg->min_distance = laser_msg.min_distance; 
                laser_mav_msg->angle = laser_msg.angle; 
                laser_mav_msg->laser_x = laser_msg.laser_x; 
                laser_mav_msg->laser_y = laser_msg.laser_y; 

                
                laser_receiver_pub.publish(laser_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::LaserReceiverPlugin, mavplugin::MavRosPlugin)
