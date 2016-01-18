/**
 * @brief Offboard control plugin
 * @file offboard.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 */
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

//#include <mavros/OffboardVision.h>
#include <mavros_vision/OffboardVision.h>

namespace mavplugin {

class OffboardPlugin : public MavRosPlugin {
public:
	OffboardPlugin()
	{ };

	void initialize(UAS &uas_)//,
			//ros::NodeHandle &nh,
			//diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		// replace service to topic
		offboard_vision_sub = nh.subscribe("offboard_vision", 10, &OffboardPlugin::offboard_vision_cb, this);
	}

	/*std::string get_name() {
		// no spaces
		return "OffboardControl";
	}*/

	//std::vector<uint8_t> get_supported_messages() {
		//return {
			// List here messages for message_rx_cb
		//};
	//}

	//void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		// no messages in get_supported_messages()
	//}
    const message_map get_rx_handlers() {
		return {
			       //MESSAGE_HANDLER(MAVLINK_MSG_ID_CLARENCE_NEW, &ClarenceNewPlugin::handle_clarence_new)
		};
	}
private:
	UAS *uas;
    ros::NodeHandle nh;
	ros::Subscriber offboard_vision_sub;

	/* -*- low-level send -*- */

	// perhaps better add send_ prefix, naming: message name in lower case
	void vision_position_estimate(float x, float y, float z, float roll, float pitch, float yaw) {

		mavlink_message_t msg;
		mavlink_msg_vision_position_estimate_pack(0, 0, &msg,
				0,
				x,
				y,
		        z,
				roll,
				pitch,
				yaw);
		//uas->mav_link->send_message(&msg);
		UAS_FCU(uas)->send_message(&msg);
	}

	/* -*- callbacks -*- */

	// Did you really need service?
	// I think topic subscriber are better for this case
	void offboard_vision_cb(const mavros_vision::OffboardVision &req) {

		vision_position_estimate(req.x, req.y, req.z, req.roll, req.pitch, req.yaw);
	}

};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::OffboardPlugin, mavplugin::MavRosPlugin)
