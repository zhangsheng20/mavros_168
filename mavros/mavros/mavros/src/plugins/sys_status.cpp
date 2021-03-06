/**
 * @brief System Status plugin
 * @file sys_status.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
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

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros/State.h>
#include <mavros/BatteryStatus.h>
#include <mavros/StreamRate.h>
#include <mavros/SetMode.h>

namespace mavplugin {

/**
 * Heartbeat status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
class HeartbeatStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HeartbeatStatus(const std::string name, size_t win_size) :
		diagnostic_updater::DiagnosticTask(name),
		window_size_(win_size),
		min_freq_(0.2),
		max_freq_(100),
		tolerance_(0.1),
		times_(win_size),
		seq_nums_(win_size),
		last_hb{}
	{
		clear();
	}

	void clear() {
		lock_guard lock(mutex);
		ros::Time curtime = ros::Time::now();
		count_ = 0;

		for (int i = 0; i < window_size_; i++)
		{
			times_[i] = curtime;
			seq_nums_[i] = count_;
		}

		hist_indx_ = 0;
	}

	void tick(mavlink_heartbeat_t &hb_struct) {
		lock_guard lock(mutex);
		count_++;
		last_hb = hb_struct;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);
		ros::Time curtime = ros::Time::now();
		int curseq = count_;
		int events = curseq - seq_nums_[hist_indx_];
		double window = (curtime - times_[hist_indx_]).toSec();
		double freq = events / window;
		seq_nums_[hist_indx_] = curseq;
		times_[hist_indx_] = curtime;
		hist_indx_ = (hist_indx_ + 1) % window_size_;

		if (events == 0) {
			stat.summary(2, "No events recorded.");
		}
		else if (freq < min_freq_ * (1 - tolerance_)) {
			stat.summary(1, "Frequency too low.");
		}
		else if (freq > max_freq_ * (1 + tolerance_)) {
			stat.summary(1, "Frequency too high.");
		}
		else {
			stat.summary(0, "Normal");
		}

		stat.addf("Events in window", "%d", events);
		stat.addf("Events since startup", "%d", count_);
		stat.addf("Duration of window (s)", "%f", window);
		stat.addf("Actual frequency (Hz)", "%f", freq);
		stat.addf("MAV Type", "%u", last_hb.type);
		stat.addf("Autopilot type", "%u", last_hb.autopilot);
		stat.addf("Autopilot base mode", "0x%02X", last_hb.base_mode);
		stat.addf("Autopilot custom mode", "0x%08X", last_hb.custom_mode);
		stat.addf("Autopilot system status", "%u", last_hb.system_status);
	}

private:
	int count_;
	std::vector<ros::Time> times_;
	std::vector<int> seq_nums_;
	int hist_indx_;
	std::recursive_mutex mutex;
	const size_t window_size_;
	const double min_freq_;
	const double max_freq_;
	const double tolerance_;
	mavlink_heartbeat_t last_hb;
};


class SystemStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	SystemStatusDiag(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		last_st{}
	{ };

	void set(mavlink_sys_status_t &st) {
		lock_guard lock(mutex);
		last_st = st;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

		if ((last_st.onboard_control_sensors_health & last_st.onboard_control_sensors_enabled)
				!= last_st.onboard_control_sensors_enabled)
			stat.summary(2, "Sensor helth");
		else
			stat.summary(0, "Normal");

		stat.addf("Sensor present", "0x%08X", last_st.onboard_control_sensors_present);
		stat.addf("Sensor enabled", "0x%08X", last_st.onboard_control_sensors_enabled);
		stat.addf("Sensor helth", "0x%08X", last_st.onboard_control_sensors_health);

		// decode sensor health mask
#define STAT_ADD_SENSOR(msg, sensor_mask)	\
		if (last_st.onboard_control_sensors_enabled & sensor_mask)	\
			stat.add(msg, (last_st.onboard_control_sensors_health & sensor_mask)? "Ok" : "Fail")

		STAT_ADD_SENSOR("Sensor 3D Gyro", MAV_SYS_STATUS_SENSOR_3D_GYRO);
		STAT_ADD_SENSOR("Sensor 3D Accel", MAV_SYS_STATUS_SENSOR_3D_ACCEL);
		STAT_ADD_SENSOR("Sensor 3D Mag", MAV_SYS_STATUS_SENSOR_3D_MAG);
		STAT_ADD_SENSOR("Sensor Abs Pressure", MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE);
		STAT_ADD_SENSOR("Sensor Diff Pressure", MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE);
		STAT_ADD_SENSOR("Sensor GPS", MAV_SYS_STATUS_SENSOR_GPS);
		STAT_ADD_SENSOR("Sensor Optical Flow", MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW);
		STAT_ADD_SENSOR("Sensor Vision Position", MAV_SYS_STATUS_SENSOR_VISION_POSITION);
		STAT_ADD_SENSOR("Sensor Laser Position", MAV_SYS_STATUS_SENSOR_LASER_POSITION);
		STAT_ADD_SENSOR("Sensor Ext Grount Truth", MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH);
		STAT_ADD_SENSOR("Sensor Ang Rate Control", MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
		STAT_ADD_SENSOR("Sensor Attitude Stab", MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION);
		STAT_ADD_SENSOR("Sensor Yaw Position", MAV_SYS_STATUS_SENSOR_YAW_POSITION);
		STAT_ADD_SENSOR("Sensor Z/Alt Control", MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
		STAT_ADD_SENSOR("Sensor X/Y Pos Control", MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL);
		STAT_ADD_SENSOR("Sensor Motor Outputs", MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
		STAT_ADD_SENSOR("Sensor RC Receiver", MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
		STAT_ADD_SENSOR("Sensor 3D Gyro 2", MAV_SYS_STATUS_SENSOR_3D_GYRO2);
		STAT_ADD_SENSOR("Sensor 3D Accel 2", MAV_SYS_STATUS_SENSOR_3D_ACCEL2);
		STAT_ADD_SENSOR("Sensor 3D Mag 2", MAV_SYS_STATUS_SENSOR_3D_MAG2);
		STAT_ADD_SENSOR("Geofence health", MAV_SYS_STATUS_GEOFENCE);
		STAT_ADD_SENSOR("AHRS health", MAV_SYS_STATUS_AHRS);
		//STAT_ADD_SENSOR("Terrain health", MAV_SYS_STATUS_TERRAIN);

#undef STAT_ADD_SENSOR

		stat.addf("CPU Load (%)", "%.1f", last_st.load / 10.0);
		stat.addf("Drop rate (%)", "%.1f", last_st.drop_rate_comm / 10.0);
		stat.addf("Errors comm", "%d", last_st.errors_comm);
		// stat.addf("Errors count #1", "%d", last_st.errors_count1);
		// stat.addf("Errors count #2", "%d", last_st.errors_count2);
		// stat.addf("Errors count #3", "%d", last_st.errors_count3);
		// stat.addf("Errors count #4", "%d", last_st.errors_count4);
	}

private:
	std::recursive_mutex mutex;
	mavlink_sys_status_t last_st;
};


class BatteryStatusDiag : public diagnostic_updater::DiagnosticTask
{
public:
	BatteryStatusDiag(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		voltage(-1.0),
		current(0.0),
		remaining(0.0),
		min_voltage(6)
	{};

	void set_min_voltage(float volt) {
		lock_guard lock(mutex);
		min_voltage = volt;
	}

	void set(float volt, float curr, float rem) {
		lock_guard lock(mutex);
		voltage = volt;
		current = curr;
		remaining = rem;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

		if (voltage < 0)
			stat.summary(2, "No data");
		else if (voltage < min_voltage)
			stat.summary(1, "Low voltage");
		else
			stat.summary(0, "Normal");

		stat.addf("Voltage", "%.2f", voltage);
		stat.addf("Current", "%.1f", current);
		stat.addf("Remaining", "%.1f", remaining * 100);
	}

private:
	std::recursive_mutex mutex;
	float voltage;
	float current;
	float remaining;
	float min_voltage;
};


class MemInfo : public diagnostic_updater::DiagnosticTask
{
public:
	MemInfo(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		freemem(-1),
		brkval(0)
	{};

	void set(uint16_t f, uint16_t b) {
		lock_guard lock(mutex);
		freemem = f;
		brkval = b;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

		if (freemem < 0)
			stat.summary(2, "No data");
		else if (freemem < 200)
			stat.summary(1, "Low mem");
		else
			stat.summary(0, "Normal");

		stat.addf("Free memory (B)", "%zd", freemem);
		stat.addf("Heap top", "0x%04X", brkval);
	}

private:
	std::recursive_mutex mutex;
	ssize_t freemem;
	uint16_t brkval;
};


class HwStatus : public diagnostic_updater::DiagnosticTask
{
public:
	HwStatus(const std::string name) :
		diagnostic_updater::DiagnosticTask(name),
		vcc(-1.0),
		i2cerr(0),
		i2cerr_last(0)
	{};

	void set(uint16_t v, uint8_t e) {
		lock_guard lock(mutex);
		vcc = v / 1000.0;
		i2cerr = e;
	}

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);

		if (vcc < 0)
			stat.summary(2, "No data");
		else if (vcc < 4.5)
			stat.summary(1, "Low voltage");
		else if (i2cerr != i2cerr_last) {
			i2cerr_last = i2cerr;
			stat.summary(1, "New I2C error");
		}
		else
			stat.summary(0, "Normal");

		stat.addf("Core voltage", "%f", vcc);
		stat.addf("I2C errors", "%zu", i2cerr);
	}

private:
	std::recursive_mutex mutex;
	float vcc;
	size_t i2cerr;
	size_t i2cerr_last;
};


/**
 * @brief System status plugin.
 * Required for most applications.
 */
class SystemStatusPlugin : public MavRosPlugin
{
public:
	SystemStatusPlugin() :
		hb_diag("Heartbeat", 10),
		mem_diag("APM Memory"),
		hwst_diag("APM Hardware"),
		sys_diag("System"),
		batt_diag("Battery")
	{};

	void initialize(
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		double conn_timeout_d;
		double conn_heartbeat_d;
		double min_voltage;
		bool disable_diag;

		nh.param("conn_timeout", conn_timeout_d, 30.0);
		nh.param("conn_heartbeat", conn_heartbeat_d, 0.0);
		nh.param("sys/min_voltage", min_voltage, 6.0);
		nh.param("sys/disable_diag", disable_diag, false);

		// heartbeat diag always enabled
		diag_updater.add(hb_diag);
		if (!disable_diag) {
			diag_updater.add(sys_diag);
			diag_updater.add(batt_diag);
#ifdef MAVLINK_MSG_ID_MEMINFO
			diag_updater.add(mem_diag);
#endif
#ifdef MAVLINK_MSG_ID_HWSTATUS
			diag_updater.add(hwst_diag);
#endif

			batt_diag.set_min_voltage(min_voltage);
		}


		// one-shot timeout timer
		timeout_timer = nh.createTimer(ros::Duration(conn_timeout_d),
				&SystemStatusPlugin::timeout_cb, this, true);
		timeout_timer.start();

		if (conn_heartbeat_d > 0.0) {
			heartbeat_timer = nh.createTimer(ros::Duration(conn_heartbeat_d),
					&SystemStatusPlugin::heartbeat_cb, this);
			heartbeat_timer.start();
		}
		
		for (unsigned int i = 0; i < uav_sys_id_list_.size(); i++)
        {
            std::string sys_id = str_sysid_ + static_cast<std::ostringstream*>( &(std::ostringstream() << uav_sys_id_list_[i]) )->str();
            state_pub_list_.push_back(nh.advertise<mavros::State>("state" + sys_id, 10));
            batt_pub_list_.push_back(nh.advertise<mavros::BatteryStatus>("battery" + sys_id, 10));
        }
		rate_srv = nh.advertiseService("set_stream_rate", &SystemStatusPlugin::set_rate_cb, this);
		mode_srv = nh.advertiseService("set_mode", &SystemStatusPlugin::set_mode_cb, this);
	}

	const std::string get_name() const {
		return "SystemStatus";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_HEARTBEAT, &SystemStatusPlugin::handle_heartbeat),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_SYS_STATUS, &SystemStatusPlugin::handle_sys_status),
			MESSAGE_HANDLER(MAVLINK_MSG_ID_STATUSTEXT, &SystemStatusPlugin::handle_statustext),
#ifdef MAVLINK_MSG_ID_MEMINFO
			MESSAGE_HANDLER(MAVLINK_MSG_ID_MEMINFO, &SystemStatusPlugin::handle_meminfo),
#endif
#ifdef MAVLINK_MSG_ID_HWSTATUS
			MESSAGE_HANDLER(MAVLINK_MSG_ID_HWSTATUS, &SystemStatusPlugin::handle_hwstatus),
#endif
		};
	}

private:
	HeartbeatStatus hb_diag;
	MemInfo mem_diag;
	HwStatus hwst_diag;
	SystemStatusDiag sys_diag;
	BatteryStatusDiag batt_diag;
	ros::Timer timeout_timer;
	ros::Timer heartbeat_timer;

	std::vector<ros::Publisher> state_pub_list_;
	std::vector<ros::Publisher> batt_pub_list_;
	ros::ServiceServer rate_srv;
	ros::ServiceServer mode_srv;

	/* -*- mid-level helpers -*- */

	/**
	 * Sent STATUSTEXT message to rosout
	 *
	 * @param[in] severity  Levels defined in common.xml
	 */
	void process_statustext_normal(uint8_t severity, std::string &text) {
		switch (severity) {
		case MAV_SEVERITY_EMERGENCY:
		case MAV_SEVERITY_ALERT:
		case MAV_SEVERITY_CRITICAL:
		case MAV_SEVERITY_ERROR:
			ROS_ERROR_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case MAV_SEVERITY_WARNING:
		case MAV_SEVERITY_NOTICE:
			ROS_WARN_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case MAV_SEVERITY_INFO:
			ROS_INFO_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case MAV_SEVERITY_DEBUG:
		default:
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: " << text);
			break;
		};

	}

	/**
	 * Send STATUSTEXT messate to rosout with APM severity levels
	 *
	 * @param[in] severity  APM levels.
	 */
	void process_statustext_apm_quirk(uint8_t severity, std::string &text) {
		switch (severity) {
		case 1: // SEVERITY_LOW
			ROS_INFO_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case 2: // SEVERITY_MEDIUM
			ROS_WARN_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		case 3: // SEVERITY_HIGH
		case 4: // SEVERITY_CRITICAL
		case 5: // SEVERITY_USER_RESPONSE
			ROS_ERROR_STREAM_NAMED("fcu", "FCU: " << text);
			break;

		default:
			ROS_DEBUG_STREAM_NAMED("fcu", "FCU: UNK(" <<
					(int)severity << "): " << text);
			break;
		};

	}

	/* -*- message handlers -*- */

	void handle_heartbeat(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	    if (false == isSysIdValid(sysid)) return;
	    UAS* uas = uas_list_[sysid];
	    
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);
		hb_diag.tick(hb);

		// update context && setup connection timeout
		uas->update_heartbeat(hb.type, hb.autopilot);
		uas->update_connection_status(true);
		timeout_timer.stop();
		timeout_timer.start();

		mavros::StatePtr state_msg = boost::make_shared<mavros::State>();
		state_msg->header.stamp = ros::Time::now();
		state_msg->armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
		state_msg->guided = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
		state_msg->mode = uas->str_mode_v10(hb.base_mode, hb.custom_mode);

        publishMessage(sysid, compid, state_pub_list_, state_msg);
	}

	void handle_sys_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_sys_status_t stat;
		mavlink_msg_sys_status_decode(msg, &stat);

		float volt = stat.voltage_battery / 1000.0;	// mV
		float curr = stat.current_battery / 100.0;	// 10 mA or -1
		float rem = stat.battery_remaining / 100.0;	// or -1

		mavros::BatteryStatusPtr batt_msg = boost::make_shared<mavros::BatteryStatus>();
		batt_msg->header.stamp = ros::Time::now();
		batt_msg->voltage = volt;
		batt_msg->current = curr;
		batt_msg->remaining = rem;

		sys_diag.set(stat);
		batt_diag.set(volt, curr, rem);
		publishMessage(sysid, compid, batt_pub_list_, batt_msg);
	}

	void handle_statustext(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
	    if (false == isSysIdValid(sysid)) return;
	    if (sysid != target_system_id_) return;
	    UAS* uas = uas_list_[sysid];
	    
		mavlink_statustext_t textm;
		mavlink_msg_statustext_decode(msg, &textm);

		std::string text(textm.text,
				strnlen(textm.text, sizeof(textm.text)));

		if (uas->is_ardupilotmega())
			process_statustext_apm_quirk(textm.severity, text);
		else
			process_statustext_normal(textm.severity, text);
	}

#ifdef MAVLINK_MSG_ID_MEMINFO
	void handle_meminfo(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_meminfo_t mem;
		mavlink_msg_meminfo_decode(msg, &mem);
		mem_diag.set(mem.freemem, mem.brkval);
	}
#endif

#ifdef MAVLINK_MSG_ID_HWSTATUS
	void handle_hwstatus(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_hwstatus_t hwst;
		mavlink_msg_hwstatus_decode(msg, &hwst);
		hwst_diag.set(hwst.Vcc, hwst.I2Cerr);
	}
#endif

	/* -*- timer callbacks -*- */

	void timeout_cb(const ros::TimerEvent &event) {
		uas_->update_connection_status(false);
	}

	void heartbeat_cb(const ros::TimerEvent &event) {
		mavlink_message_t msg;
		mavlink_msg_heartbeat_pack_chan(UAS_PACK_CHAN(uas_), &msg,
				MAV_TYPE_ONBOARD_CONTROLLER,
				MAV_AUTOPILOT_INVALID,
				MAV_MODE_MANUAL_ARMED,
				0,
				MAV_STATE_ACTIVE,
				0//!!
				);

		UAS_FCU(uas_)->send_message(&msg);
	}

	/* -*- ros callbacks -*- */

	bool set_rate_cb(mavros::StreamRate::Request &req,
			mavros::StreamRate::Response &res) {

		mavlink_message_t msg;
		mavlink_msg_request_data_stream_pack_chan(UAS_PACK_CHAN(uas_), &msg,
				UAS_PACK_TGT(uas_),
				req.stream_id,
				req.message_rate,
				(req.on_off)? 1 : 0
				);

		UAS_FCU(uas_)->send_message(&msg);
		return true;
	}

	bool set_mode_cb(mavros::SetMode::Request &req,
			mavros::SetMode::Response &res) {
		mavlink_message_t msg;
		uint8_t base_mode = req.base_mode;
		uint32_t custom_mode = 0;

		if (req.custom_mode != "") {
			if (!uas_->cmode_from_str(req.custom_mode, custom_mode)) {
				res.success = false;
				return true;
			}

			base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
		}

		mavlink_msg_set_mode_pack_chan(UAS_PACK_CHAN(uas_), &msg,
				uas_->get_tgt_system(),
				base_mode,
				custom_mode);
		UAS_FCU(uas_)->send_message(&msg);
		res.success = true;
		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SystemStatusPlugin, mavplugin::MavRosPlugin)

