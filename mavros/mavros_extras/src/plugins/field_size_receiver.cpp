#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/FieldSize.h>


//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class FieldSizeReceiverPlugin : public MavRosPlugin{

public:
        FieldSizeReceiverPlugin():
                field_size_receiver_nh("~field_size_receiver"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		field_size_receiver_nh.param<std::string>("frame_id", frame_id, "field_size_receiver");
        field_size_receiver_pub = field_size_receiver_nh.advertise<mavros_extras::FieldSize>("field_size_receiver", 5); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_FIELD_SIZE, &FieldSizeReceiverPlugin::handle_field_size)
		};
	}

private:
        ros::NodeHandle field_size_receiver_nh;
        ros::Publisher field_size_receiver_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_field_size(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_field_size_t field_msg;
		mavlink_msg_field_size_decode(msg, &field_msg);//decode 
                
               
                auto field_mav_msg = boost::make_shared<mavros_extras::FieldSize>();//define a msg 
                field_mav_msg->header.stamp = ros::Time::now();

                field_mav_msg->length = field_msg.length; 
                field_mav_msg->width = field_msg.width; 
                field_mav_msg->height = field_msg.height; 
                field_mav_msg->times = field_msg.times; 

                
                field_size_receiver_pub.publish(field_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::FieldSizeReceiverPlugin, mavplugin::MavRosPlugin)
