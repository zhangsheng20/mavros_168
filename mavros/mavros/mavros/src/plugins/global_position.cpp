/**
 * @brief Global Position plugin
 * @file global_position.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
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
#include <mavros/gps_conversions.h>
#include <pluginlib/class_list_macros.h>

#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

namespace mavplugin {

/**
 * @brief Global position plugin.
 *
 * Publishes global position. Convertion from GPS to UTF allows
 * publishing local position to TF and PoseWithCovarianceStamped.
 *
 */
class GlobalPositionPlugin : public MavRosPlugin {
public:
	GlobalPositionPlugin() :
		send_tf(false),
		rot_cov(99999.0)
	{ };

	void initialize(
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		gp_nh = ros::NodeHandle(nh, "global_position");

		gp_nh.param("send_tf", send_tf, true);
		gp_nh.param<std::string>("frame_id", frame_id, "local_origin");
		gp_nh.param<std::string>("child_frame_id", child_frame_id, "fcu");
		gp_nh.param<double>("rot_covariance", rot_cov, 99999.0);

		for (unsigned int i = 0; i < uav_sys_id_list_.size(); i++)
        {
            std::string sys_id = str_sysid_ + static_cast<std::ostringstream*>( &(std::ostringstream() << uav_sys_id_list_[i]) )->str();
            fix_pub_list_.push_back(gp_nh.advertise<sensor_msgs::NavSatFix>("global" + sys_id, 10));
            pos_pub_list_.push_back(gp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("local" + sys_id, 10));
            vel_pub_list_.push_back(gp_nh.advertise<geometry_msgs::Vector3Stamped>("gps_vel" + sys_id, 10));
            rel_alt_pub_list_.push_back(gp_nh.advertise<std_msgs::Float64>("rel_alt" + sys_id, 10));
            hdg_pub_list_.push_back(gp_nh.advertise<std_msgs::Float64>("compass_hdg" + sys_id, 10));
            
            beacon_fix_pub_list_.push_back(gp_nh.advertise<sensor_msgs::NavSatFix>("beacon_global" + sys_id, 10));
            beacon_pos_pub_list_.push_back(gp_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("beacon_local" + sys_id, 10));
            beacon_vel_pub_list_.push_back(gp_nh.advertise<geometry_msgs::Vector3Stamped>("beacon_gps_vel" + sys_id, 10));
            beacon_rel_alt_pub_list_.push_back(gp_nh.advertise<std_msgs::Float64>("beacon_rel_alt" + sys_id, 10));
            beacon_hdg_pub_list_.push_back(gp_nh.advertise<std_msgs::Float64>("beacon_compass_hdg" + sys_id, 10));
        }

	}

	std::string const get_name() const {
		return "GlobalPosition";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &GlobalPositionPlugin::handle_global_position_int)
		};
	}

private:
	ros::NodeHandle gp_nh;
	std::vector<ros::Publisher> fix_pub_list_;
	std::vector<ros::Publisher> pos_pub_list_;
	std::vector<ros::Publisher> vel_pub_list_;
	std::vector<ros::Publisher> hdg_pub_list_;
	std::vector<ros::Publisher> rel_alt_pub_list_;

	std::vector<ros::Publisher> beacon_fix_pub_list_;
	std::vector<ros::Publisher> beacon_pos_pub_list_;
	std::vector<ros::Publisher> beacon_vel_pub_list_;
	std::vector<ros::Publisher> beacon_hdg_pub_list_;
	std::vector<ros::Publisher> beacon_rel_alt_pub_list_;

	tf::TransformBroadcaster tf_broadcaster;

	std::string frame_id;		//!< origin for TF
	std::string child_frame_id;	//!< frame for TF and Pose
	bool send_tf;
	double rot_cov;

	/**
	 * @todo Handler for GLOBAL_POSITION_INT_COV
	 */

	void handle_global_position_int(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_global_position_int_t gp_pos;
		mavlink_msg_global_position_int_decode(msg, &gp_pos);

        if (false == isSysIdValid(sysid)) return;
        UAS* uas = uas_list_[sysid];

		ROS_DEBUG_THROTTLE_NAMED(10, "global", "Global position: boot_ms:%06d "
				"lat/lon/alt:(%d %d %d) relative_alt: (%d) gps_vel:(%d %d %d) compass_heading: (%d)",
				gp_pos.time_boot_ms,
				gp_pos.lat, gp_pos.lon, gp_pos.alt, gp_pos.relative_alt,
				gp_pos.vx, gp_pos.vy, gp_pos.vz, gp_pos.hdg);

		geometry_msgs::PoseWithCovarianceStampedPtr pose_cov =
			boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

		std_msgs::Header header;
		header.frame_id = frame_id;
		header.stamp = ros::Time::now();

		sensor_msgs::NavSatFixPtr gps_cord =
			boost::make_shared<sensor_msgs::NavSatFix>();
		geometry_msgs::Vector3StampedPtr gps_vel =
			boost::make_shared<geometry_msgs::Vector3Stamped>();
		std_msgs::Float64Ptr relative_alt = boost::make_shared<std_msgs::Float64>();
		std_msgs::Float64Ptr compass_heading = boost::make_shared<std_msgs::Float64>();

		gps_cord->header = header;
		gps_cord->latitude = gp_pos.lat / 1E7;
		gps_cord->longitude = gp_pos.lon / 1E7;
		gps_cord->altitude = gp_pos.alt / 1E3; // in meters

		// fill GPS status fields
		gps_cord->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
		if (uas->get_gps_status())
		{
            gps_cord->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
		}
		else
		{
		    gps_cord->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
		}

		// try compute LLA covariance from GPS_RAW_INT data
		double hdop = uas->get_gps_eph();
		if (!std::isnan(hdop)) {
			double hdop2 = std::pow(hdop, 2);

			// From nmea_navsat_driver
			gps_cord->position_covariance[0] = hdop2;
			gps_cord->position_covariance[4] = hdop2;
			gps_cord->position_covariance[8] = std::pow(2 * hdop, 2);
			gps_cord->position_covariance_type =
				sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
		}
		else {
			gps_cord->position_covariance_type =
				sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
		}

		gps_vel->header = header;
		gps_vel->vector.x = gp_pos.vx / 1E2; // in m/s
		gps_vel->vector.y = gp_pos.vy / 1E2;
		gps_vel->vector.z = gp_pos.vz / 1E2;

		relative_alt->data = gp_pos.relative_alt / 1E3; // in meters
		if (gp_pos.hdg != UINT16_MAX)
			compass_heading->data = gp_pos.hdg / 1E2; // in degrees

		double northing, easting;
		std::string zone;

		/** Adapted from gps_umd ROS package @a http://wiki.ros.org/gps_umd
		 *  Author: Ken Tossell <ken AT tossell DOT net>
		 */
		UTM::LLtoUTM(gps_cord->latitude, gps_cord->longitude, northing, easting, zone);

		pose_cov->header = header;
		pose_cov->pose.pose.position.x = easting;
		pose_cov->pose.pose.position.y = northing;
		pose_cov->pose.pose.position.z = gp_pos.relative_alt / 1E3;

		geometry_msgs::Quaternion q_aux;
		tf::Quaternion q(uas->get_attitude_orientation());
		tf::quaternionTFToMsg(q, q_aux);
		pose_cov->pose.pose.orientation = q_aux;

		// Use ENU covariance to build XYZRPY covariance
		boost::array<double, 36> covariance = {
			gps_cord->position_covariance[0],
			gps_cord->position_covariance[1],
			gps_cord->position_covariance[2],
			0, 0, 0,
			gps_cord->position_covariance[3],
			gps_cord->position_covariance[4],
			gps_cord->position_covariance[5],
			0, 0, 0,
			gps_cord->position_covariance[6],
			gps_cord->position_covariance[7],
			gps_cord->position_covariance[8],
			0, 0, 0,
			0, 0, 0, rot_cov, 0, 0,
			0, 0, 0, 0, rot_cov, 0,
			0, 0, 0, 0, 0, rot_cov
		};

		pose_cov->pose.covariance = covariance;

        if (true)
        {
            publishMessage(sysid, compid, fix_pub_list_, gps_cord);
            publishMessage(sysid, compid, pos_pub_list_, pose_cov);
            publishMessage(sysid, compid, vel_pub_list_, gps_vel);
            publishMessage(sysid, compid, rel_alt_pub_list_, relative_alt);
            if (gp_pos.hdg != UINT16_MAX)
            {
                publishMessage(sysid, compid, hdg_pub_list_, compass_heading);
            }
		}
		else if(false)
		{
		    publishMessage(sysid, compid, beacon_fix_pub_list_, gps_cord);
		    publishMessage(sysid, compid, beacon_pos_pub_list_, pose_cov);
		    publishMessage(sysid, compid, beacon_vel_pub_list_, gps_vel);
		    publishMessage(sysid, compid, beacon_rel_alt_pub_list_, relative_alt);
			if (gp_pos.hdg != UINT16_MAX)
            {
                publishMessage(sysid, compid, beacon_hdg_pub_list_, compass_heading);
            }
		}
		if (send_tf) {
			tf::Transform transform;
			// XXX: we realy need change frame?
			transform.setOrigin(tf::Vector3(pose_cov->pose.pose.position.y,
						pose_cov->pose.pose.position.x,
						-pose_cov->pose.pose.position.z));

			transform.setRotation(uas->get_attitude_orientation());

			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose_cov->header.stamp,
						frame_id, child_frame_id));
		}
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::GlobalPositionPlugin, mavplugin::MavRosPlugin)
