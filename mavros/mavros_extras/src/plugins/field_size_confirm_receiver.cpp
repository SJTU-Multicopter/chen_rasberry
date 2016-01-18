#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/FieldSizeConfirm.h>


//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class FieldSizeConfirmReceiverPlugin : public MavRosPlugin{

public:
        FieldSizeConfirmReceiverPlugin():
                field_size_confirm_receiver_nh("~field_size_confirm_receiver"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		field_size_confirm_receiver_nh.param<std::string>("frame_id", frame_id, "field_size_confirm_receiver");
        field_size_confirm_receiver_pub = field_size_confirm_receiver_nh.advertise<mavros_extras::FieldSizeConfirm>("field_size_confirm_receiver", 100); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_FIELD_SIZE_CONFIRM, &FieldSizeConfirmReceiverPlugin::handle_field_size_confirm)
		};
	}

private:
        ros::NodeHandle field_size_confirm_receiver_nh;
        ros::Publisher field_size_confirm_receiver_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_field_size_confirm(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_field_size_confirm_t field_confirm_msg;
		mavlink_msg_field_size_confirm_decode(msg, &field_confirm_msg);//decode 
                
               
                auto field_confirm_mav_msg = boost::make_shared<mavros_extras::FieldSizeConfirm>();//define a msg 
                field_confirm_mav_msg->header.stamp = ros::Time::now();

                field_confirm_mav_msg->length = field_confirm_msg.length; 
                field_confirm_mav_msg->width = field_confirm_msg.width; 
                field_confirm_mav_msg->height = field_confirm_msg.height; 
                field_confirm_mav_msg->times = field_confirm_msg.times; 
                field_confirm_mav_msg->confirm = field_confirm_msg.confirm; 
                
                field_size_confirm_receiver_pub.publish(field_confirm_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::FieldSizeConfirmReceiverPlugin, mavplugin::MavRosPlugin)
