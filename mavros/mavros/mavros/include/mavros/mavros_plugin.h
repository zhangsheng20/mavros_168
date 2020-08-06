/**
 * @brief MAVROS Plugin base
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
 */
/*
 * Copyright 2013 Vladimir Ermakov.
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

#pragma once

#include <map>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <mavros/mavros_uas.h>

namespace mavplugin {
using mavros::UAS;
typedef std::lock_guard<std::recursive_mutex> lock_guard;
typedef std::unique_lock<std::recursive_mutex> unique_lock;
typedef mavros::uas_array_t uas_array_t;

/**
 * @brief Helper macros to define message handler map item
 */
#define MESSAGE_HANDLER(_message_id, _class_method_ptr)	\
	{ _message_id, boost::bind(_class_method_ptr, this, _1, _2, _3) }

/**
 * @brief MAVROS Plugin base class
 */
class MavRosPlugin
{
private:
	MavRosPlugin(const MavRosPlugin&) = delete;

public:
	typedef boost::function<void(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid)>
		message_handler;
	typedef std::map<uint8_t, message_handler> message_map;
	// pluginlib return boost::shared_ptr
	typedef boost::shared_ptr<MavRosPlugin> Ptr;
	typedef boost::shared_ptr<MavRosPlugin const> ConstPtr;

	virtual ~MavRosPlugin() {};
	
	void init(UAS* uas, const uas_array_t& uas_list, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater)
	{
	    uas_ = uas;
	    uas_list_ = uas_list;
	    initialize(nh, diag_updater);
	}

	/**
	 * @brief Plugin initializer
	 *
	 * @param[in] uas           UAS instance (handles FCU connection and some statuses)
	 * @param[in] nh            main mavros ros::NodeHandle (private)
	 * @param[in] diag_updater  main diagnostic updater
	 */
	virtual void initialize(
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater) = 0;

	/**
	 * @brief Plugin name (CamelCase)
	 */
	virtual const std::string get_name() const = 0;

	/**
	 * @brief Return map with message rx handlers
	 */
	virtual const message_map get_rx_handlers() = 0;

protected:
	/**
	 * @brief Plugin constructor
	 */
	MavRosPlugin():
	    uas_(nullptr)
	{
        ros::NodeHandle private_nh_("~");
        // private_nh_.getParam("uav_sys_id_list", uav_sys_id_list_);
        private_nh_.getParam("uav_comp_id_list", uav_comp_id_list_);

        private_nh_.getParam("system_id", system_id);
        uav_sys_id_list_.push_back(system_id);
        uav_sys_id_list_.push_back(255);

        //TODO: check if positive
        
        str_sysid_ = "/sys_id_";
    }
    
    template<class T>
    void publishMessage(uint8_t sysid, uint8_t compid, const std::vector<ros::Publisher>& publisher_list, const T msg)
    {
        for (uint8_t i = 0; i < uav_sys_id_list_.size(); i++)
        {
            if (sysid == uav_sys_id_list_[i])
            {
                if (publisher_list[i].getNumSubscribers() > 0)
                {
                    publisher_list[i].publish(msg);
                }
                break;
            }
        }
    }
    
    bool isSysIdValid(uint8_t sysid)
    {
        bool rt = false;
        if (sysid < 1 || sysid > 255)
        {
            rt = false;
        }
        else if (NULL == uas_list_[sysid])
        {
            rt = false;
        }
        else
        {
            rt = true;
        }
        return rt;
    }
protected:
    std::vector<int> uav_sys_id_list_;
    std::vector<int> uav_comp_id_list_;
    int system_id;
    uint8_t target_system_id_;
    uas_array_t uas_list_;
    UAS* uas_;
    
    std::string str_sysid_;
};

}; // namespace mavplugin
