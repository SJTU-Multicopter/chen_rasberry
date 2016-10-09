#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/SonarDistance.h>
#include "std_msgs/Float32.h" 

//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class SonarReceiverPlugin : public MavRosPlugin{

public:
        SonarReceiverPlugin():
                sonar_receiver_nh("~sonar_receiver"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
	{
		uas = &uas_;

		// general params
		sonar_receiver_nh.param<std::string>("frame_id", frame_id, "sonar_receiver");
        sonar_receiver_pub = sonar_receiver_nh.advertise<mavros_extras::SonarDistance>("sonar_receiver", 4); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
			      MESSAGE_HANDLER(MAVLINK_MSG_ID_SONAR_DISTANCE, &SonarReceiverPlugin::handle_sonar_distance)
		};
	}

private:
        ros::NodeHandle sonar_receiver_nh;
        ros::Publisher sonar_receiver_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_sonar_distance(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_sonar_distance_t sonar_msg;
		mavlink_msg_sonar_distance_decode(msg, &sonar_msg);//decode 
                
                //auto header = uas->synchronized_header(frame_id,clarence_msg.mavros_a);//add  header,XXXX

                auto sonar_mav_msg = boost::make_shared<mavros_extras::SonarDistance>();//define a msg the same type as  mavros_extras::ClarenceNewMavros

                sonar_mav_msg->sonar_front = sonar_msg.sonar_front; 
                sonar_mav_msg->sonar_behind = sonar_msg.sonar_behind; 
                sonar_mav_msg->sonar_left = sonar_msg.sonar_left; 
                sonar_mav_msg->sonar_right = sonar_msg.sonar_right; 
                sonar_mav_msg->sonar_up = sonar_msg.sonar_up; 
                sonar_mav_msg->sonar_down = sonar_msg.sonar_down; 
                sonar_mav_msg->sonar_cam = sonar_msg.sonar_cam; 
                
                sonar_receiver_pub.publish(sonar_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::SonarReceiverPlugin, mavplugin::MavRosPlugin)
