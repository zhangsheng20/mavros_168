/**
 * @brief Rangefinder plugin
 * @file rangefinder.cpp
 * @author Pierre Kancir <khancyr@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Ardupilot.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Range.h>

namespace mavplugin {

class RangefinderPlugin : public MavRosPlugin {
public:
	RangefinderPlugin() :
        has_dist_sensor(false)
	{ };

    void initialize(
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
        for (unsigned int i = 0; i < uav_sys_id_list_.size(); i++)
        {
            std::string sys_id = str_sysid_ + static_cast<std::ostringstream*>( &(std::ostringstream() << uav_sys_id_list_[i]) )->str();
            rangefinder_pub_list_.push_back(nh.advertise<sensor_msgs::Range>("sensor/rangefinder" + sys_id, 10));
        }
	}

    std::string const get_name() const {
		return "RangeFinderPub";
	}

	const message_map get_rx_handlers()
	{
		return {
            MESSAGE_HANDLER(MAVLINK_MSG_ID_DISTANCE_SENSOR, &RangefinderPlugin::handle_rangefinder)
		};
	}

private:
	ros::NodeHandle rangefinder_nh;

	std::vector<ros::Publisher> rangefinder_pub_list_;

    bool has_dist_sensor;


	void handle_rangefinder(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_distance_sensor_t dist_sensor;
        mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

        ROS_INFO_COND_NAMED(!has_dist_sensor, "rangefinder", "Rangefinder sensor detected!");
        has_dist_sensor = true;

		sensor_msgs::RangePtr rangefinder_msg = boost::make_shared<sensor_msgs::Range>();
		rangefinder_msg->header.stamp = ros::Time::now();
		rangefinder_msg->header.frame_id = "/rangefinder";
		rangefinder_msg->radiation_type = dist_sensor.type;
		rangefinder_msg->field_of_view = 0;
		rangefinder_msg->min_range = dist_sensor.min_distance;
		rangefinder_msg->max_range = dist_sensor.max_distance;
        rangefinder_msg->range = dist_sensor.current_distance/100.0f;

		publishMessage(sysid, compid, rangefinder_pub_list_, rangefinder_msg);
	}
};
}	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::RangefinderPlugin, mavplugin::MavRosPlugin)
