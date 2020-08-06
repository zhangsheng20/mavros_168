/**
 * @brief MAVROS GCS proxy
 * @file gcs_bridge.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014 Vladimir Ermakov.
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

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros/utils.h>
#include <mavconn/interface.h>

#include <mavros/Mavlink.h>
#define GCS_ID 255
#define OTHER_UAV_ID 2

using namespace mavros;
using namespace mavconn;

ros::Publisher mavlink_pub;
ros::Subscriber mavlink_sub;
MAVConnInterface::Ptr gcs_link;
int local_pos_recv_count = 0;
int att_recv_count = 0;
int global_pos_recv_count = 0;

bool check_need_transfer_to_pixhawk(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid)
{
	if(sysid!=GCS_ID)
	{
		return false;
	}
	return true;
}

bool check_need_transfer_to_gcs(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid)
{
	if(mmsg->msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED)
	{
		local_pos_recv_count++;
		if(local_pos_recv_count % 4 == 0) //down sample
		{

			return true;

		}
		else
			return false;

	}

	if(mmsg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
	{

		global_pos_recv_count++;
		if(global_pos_recv_count % 4 == 0) //down sample
		{

			return true;

		}
		else
			return false;

	}
	if(mmsg->msgid == MAVLINK_MSG_ID_ATTITUDE)
	{
		att_recv_count++;
		if(att_recv_count % 4 == 0) //down sample
		{
			return true;
		}
		else
			return false;

	}
	return true;

}


/*transfer data from gcs to pixhawk  */
void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {

	if(check_need_transfer_to_pixhawk(mmsg, sysid,compid))
	{
		MavlinkPtr rmsg = boost::make_shared<Mavlink>();
		rmsg->header.stamp = ros::Time::now();
		mavutils::copy_mavlink_to_ros(mmsg, rmsg);
		mavlink_pub.publish(rmsg);
	}
};


/*transfer data from pixhawk to gcs  */
void mavlink_sub_cb(const Mavlink::ConstPtr &rmsg) {
	mavlink_message_t mmsg;
//	printf("sys id is %d, component id is %d",rmsg->sysid, rmsg->compid);
	if (mavutils::copy_ros_to_mavlink(rmsg, mmsg))
	{
		if(check_need_transfer_to_gcs(&mmsg,rmsg->sysid,rmsg->compid))
		gcs_link->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	}
	else
		ROS_ERROR("Packet drop: illegal payload64 size");
};

int main(int argc, char *argv[])
{

	printf("GCS_BRIDGE START!\n");
	ros::init(argc, argv, "gcs_bridge");
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle mavlink_nh("/mavlink");

	std::string gcs_url;
	priv_nh.param<std::string>("gcs_url", gcs_url, "udp://@");
	printf("[GCS_BRIDGE] %s !\n",gcs_url.c_str());
	try {
		gcs_link = MAVConnInterface::open_url(gcs_url);
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("GCS: %s", ex.what());
		return 1;
	}

	mavlink_pub = mavlink_nh.advertise<Mavlink>("to", 10);
	gcs_link->message_received.connect(mavlink_pub_cb);
//	gcs_link->message_received.connect(boost::bind(&MavRos::plugin_route_cb, this, _1, _2, _3));

	mavlink_sub = mavlink_nh.subscribe("from", 10, mavlink_sub_cb,
			ros::TransportHints()
				.unreliable()
				.maxDatagramSize(1024));

	ros::spin();
	return 0;
}

