#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_extras/PumpStatus.h>

//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class PumpStatusPlugin : public MavRosPlugin{

public:
        PumpStatusPlugin():
                pump_status_nh("~pump_status"),
                uas(nullptr)
                
        { };

        void initialize(UAS &uas_)
    {
        uas = &uas_;

        // general params
        pump_status_nh.param<std::string>("frame_id", frame_id, "pump_status");
        pump_status_pub = pump_status_nh.advertise<mavros_extras::PumpStatus>("pump_status", 2); //add publisher to handler
    }

    const message_map get_rx_handlers() {
        return {
                  MESSAGE_HANDLER(MAVLINK_MSG_ID_PUMP_STATUS, &PumpStatusPlugin::handle_pump_status)
        };
    }

private:
        ros::NodeHandle pump_status_nh;
        ros::Publisher pump_status_pub;
        UAS *uas; 
        std::string frame_id;
        

        void handle_pump_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_pump_status_t pump_msg;
        mavlink_msg_pump_status_decode(msg, &pump_msg);//decode 
                
                //auto header = uas->synchronized_header(frame_id,clarence_msg.mavros_a);//add  header,XXXX

                auto pump_mav_msg = boost::make_shared<mavros_extras::PumpStatus>();//define a msg the same type as  mavros_extras::ClarenceNewMavros

                pump_mav_msg->pump_speed = pump_msg.pump_speed; 
                pump_mav_msg->spray_speed = pump_msg.spray_speed; 
                
                pump_status_pub.publish(pump_mav_msg); //publish msg

               }
       
            
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::PumpStatusPlugin, mavplugin::MavRosPlugin)
