#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/ExtraFunctionReceiver.h>


//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class ExtraFunctionReceiverPlugin : public MavRosPlugin{

public:
        ExtraFunctionReceiverPlugin():
                extra_function_receiver_nh("~extra_function_receiver"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		extra_function_receiver_nh.param<std::string>("frame_id", frame_id, "extra_function_receiver");
                    extra_function_receiver_pub = extra_function_receiver_nh.advertise<mavros_extras::ExtraFunctionReceiver>("extra_function_receiver", 5); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_EXTRA_FUNCTION, &ExtraFunctionReceiverPlugin::handle_extra_function)
		};
	}

private:
        ros::NodeHandle extra_function_receiver_nh;
        ros::Publisher extra_function_receiver_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_extra_function(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
                mavlink_extra_function_t extra_msg;
                mavlink_msg_extra_function_decode(msg, &extra_msg);//decode 
                
               
                auto extra_mav_msg = boost::make_shared<mavros_extras::ExtraFunctionReceiver>();//define a msg 
                extra_mav_msg->header.stamp = ros::Time::now();

                extra_mav_msg->obs_avoid_enable = extra_msg.obs_avoid_enable; 
                extra_mav_msg->laser_height_enable = extra_msg.laser_height_enable; 
                extra_mav_msg->add_one = extra_msg.add_one;
                extra_mav_msg->add_two = extra_msg.add_two;
                extra_mav_msg->add_three = extra_msg.add_three;
                
                extra_function_receiver_pub.publish(extra_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::ExtraFunctionReceiverPlugin, mavplugin::MavRosPlugin)
