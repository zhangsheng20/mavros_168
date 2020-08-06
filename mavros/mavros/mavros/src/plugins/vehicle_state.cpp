/**
 * @brief Vehicle state plugin
 * @file vehicle_state.cpp
 * @author Yuchao Hu <hycwuy@gmail.com>
 *
 * @addtogroup plugin
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/GeoCoords.hpp>
#include <common_msgs/state.h>
#include "common/EarthUtils.hpp"
#include "mavros/utils.h"
#include "nus_msgs/StateWithCovarianceStamped.h"
#include <std_srvs/Trigger.h> 
// #include "nus_msgs/StateWithCovarianceMaskStamped.h"


namespace mavplugin
{

class VehicleStatePlugin : public MavRosPlugin
{
public:
    VehicleStatePlugin()
    {}



    void initialize(
        ros::NodeHandle &nh,
        diagnostic_updater::Updater &diag_updater)
    {
        vehicle_state_nh_ = ros::NodeHandle(nh, "vehicle_state");
        engine0_srv_ = nh.advertiseService("engine0", &VehicleStatePlugin::engine0_cb, this);
        vehicle_state_nh_.param("enable_hil", p_enable_hil_, false);
        nh.param("target_system_id", p_target_system_id_, 1);
        nh.param("target_component_id", p_target_component_id_, 1);
        state_measurement_sub_ = vehicle_state_nh_.subscribe("/state/measurement", 10, &VehicleStatePlugin::StateMeasurementCallback, this);
        state_reference_sub_ = vehicle_state_nh_.subscribe("/state/reference", 10, &VehicleStatePlugin::StateReferenceCallback, this);
        if (p_enable_hil_)
        {
            state_ground_truth_sub_           = vehicle_state_nh_.subscribe("/hil/state/ground_truth", 10, &VehicleStatePlugin::StateGroundTruthCallback, this);
            sensor_imu_sub_                   = vehicle_state_nh_.subscribe("/hil/sensor/imu", 50, &VehicleStatePlugin::SensorImuCallback, this);
            sensor_magnetic_field_sub_        = vehicle_state_nh_.subscribe("/hil/sensor/magnetic_field", 10, &VehicleStatePlugin::SensorMagneticFieldCallback, this);
            sensor_absolute_pressure_sub_     = vehicle_state_nh_.subscribe("/hil/sensor/absolute_pressure", 10, &VehicleStatePlugin::SensorAbsolutePressureCallback, this);
            sensor_differential_pressure_sub_ = vehicle_state_nh_.subscribe("/hil/sensor/differential_pressure", 10, &VehicleStatePlugin::SensorDifferentialPressureCallback, this);
            sensor_pressure_altitude_sub_     = vehicle_state_nh_.subscribe("/hil/sensor/pressure_altitude", 10, &VehicleStatePlugin::SensorPressureAltitudeCallback, this);
            sensor_temperature_sub_           = vehicle_state_nh_.subscribe("/hil/sensor/temperature", 10, &VehicleStatePlugin::SensorTemperatureCallback, this);
            sensor_gps_sub_                   = vehicle_state_nh_.subscribe("/hil/sensor/gps", 10, &VehicleStatePlugin::SensorGpsCallback, this);
        }
    }

    const std::string get_name() const {
        return "VehicleState";
    }

    const message_map get_rx_handlers() {
        return { /* Rx disabled */ };
    }

private:
    ros::NodeHandle vehicle_state_nh_;

    ros::Subscriber state_measurement_sub_;
    ros::Subscriber state_reference_sub_;
    ros::Subscriber state_ground_truth_sub_;
    ros::Subscriber sensor_imu_sub_;
    ros::Subscriber sensor_magnetic_field_sub_;
    ros::Subscriber sensor_absolute_pressure_sub_;
    ros::Subscriber sensor_differential_pressure_sub_;
    ros::Subscriber sensor_pressure_altitude_sub_;
    ros::Subscriber sensor_temperature_sub_;
    ros::Subscriber sensor_gps_sub_;
    ros::ServiceServer engine0_srv_;

    bool p_enable_hil_;
    int p_target_system_id_;
    int p_target_component_id_;
    
    mavlink_hil_sensor_t hil_sensor_msg;
    mavlink_hil_state_quaternion_t hil_state_quaternion_msg;
        // pose
    mavlink_vision_position_estimate_t vision_position_estimate_msg;
    mavlink_vision_speed_estimate_t vision_speed_estimate_msg;

    void StateMeasurementCallback(const nus_msgs::StateWithCovarianceStamped::ConstPtr msg_ptr) 
    {
        {
            
            vision_position_estimate_msg.usec = msg_ptr->header.stamp.toNSec() / 1000;
            // from NWU to NED
            vision_position_estimate_msg.x =  msg_ptr->pose.pose.position.x;
            vision_position_estimate_msg.y = -msg_ptr->pose.pose.position.y;
            vision_position_estimate_msg.z = -msg_ptr->pose.pose.position.z;
            tf::Quaternion q(msg_ptr->pose.pose.orientation.x, msg_ptr->pose.pose.orientation.y, msg_ptr->pose.pose.orientation.z, msg_ptr->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            vision_position_estimate_msg.roll  = roll;
            vision_position_estimate_msg.pitch = -pitch;
            vision_position_estimate_msg.yaw   = -yaw;

            // convert to mavlink
            mavlink_message_t mavmsg;
            mavlink_msg_vision_position_estimate_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &vision_position_estimate_msg);
            UAS_FCU(uas_)->send_message(&mavmsg);
        }

        {
            // velocity
            
            
            vision_speed_estimate_msg.usec = msg_ptr->header.stamp.toNSec() / 1000;
            // from NWU to NED
            vision_speed_estimate_msg.x =  msg_ptr->velocity.twist.linear.x;
            vision_speed_estimate_msg.y = -msg_ptr->velocity.twist.linear.y;
            vision_speed_estimate_msg.z = -msg_ptr->velocity.twist.linear.z;

            mavlink_message_t mavmsg;
            mavlink_msg_vision_speed_estimate_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &vision_speed_estimate_msg);
            UAS_FCU(uas_)->send_message(&mavmsg);
        }
    }

    bool engine0_cb(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response){
        ROS_INFO("engine0 is activated");
        response.success = true;  
        response.message = "engine0 is activated";

        mavlink_set_position_target_local_ned_t target_msg;
        // from NWU to NED
        target_msg.time_boot_ms     = vision_position_estimate_msg.usec;
        target_msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
        //Bitmask to indicate which dimensions should be ignored by the vehicle:
        // a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. 
        //If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. 
        //Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        //IDLE: 0b0000111111111111
        //work: 0b0000000000000000
        target_msg.type_mask= 0b0000111111111111;
        target_msg.x        =  vision_position_estimate_msg.x;
        target_msg.y        = vision_position_estimate_msg.y;
        target_msg.z        = vision_position_estimate_msg.z;
        target_msg.vx       = vision_speed_estimate_msg.x;
        target_msg.vy       = vision_speed_estimate_msg.y;
        target_msg.vz       = vision_speed_estimate_msg.z;
        target_msg.afx      = 0;
        target_msg.afy      = 0;
        target_msg.afz      = 0;
        target_msg.yaw      = vision_position_estimate_msg.yaw;
        target_msg.yaw_rate = 0;
        target_msg.target_system = static_cast<uint8_t>(p_target_system_id_);
        target_msg.target_component = static_cast<uint8_t>(p_target_component_id_);

        mavlink_message_t mavmsg;
        mavlink_msg_set_position_target_local_ned_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &target_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
        return true;
    }

    void StateReferenceCallback(const common_msgs::state::ConstPtr msg_ptr) const
    {
        mavlink_set_position_target_local_ned_t target_msg;

        // from NWU to NED
        target_msg.time_boot_ms     = msg_ptr->header.stamp.toNSec() / 1000000;
        target_msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
        //Bitmask to indicate which dimensions should be ignored by the vehicle:
        // a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. 
        //If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. 
        //Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
        //IDLE: 0b0000111111111111
        //work: 0b0000000000000000
        target_msg.type_mask= 0b0000000000000000;
        target_msg.x        =  msg_ptr->pos.x;
        target_msg.y        = -msg_ptr->pos.y;
        target_msg.z        = -msg_ptr->pos.z;
        target_msg.vx       =  msg_ptr->vel.x;
        target_msg.vy       = -msg_ptr->vel.y;
        target_msg.vz       = -msg_ptr->vel.z;
        target_msg.afx      =  msg_ptr->acc.x;
        target_msg.afy      = -msg_ptr->acc.y;
        target_msg.afz      = -msg_ptr->acc.z;
        target_msg.yaw      = -msg_ptr->yaw.x;
        target_msg.yaw_rate = -msg_ptr->yaw.y;
        target_msg.target_system = static_cast<uint8_t>(p_target_system_id_);
        target_msg.target_component = static_cast<uint8_t>(p_target_component_id_);

        mavlink_message_t mavmsg;
        mavlink_msg_set_position_target_local_ned_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &target_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }

    void StateGroundTruthCallback(const nus_msgs::StateWithCovarianceStamped::ConstPtr msg_ptr)
    {
        hil_state_quaternion_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        // from NWU to NED
        tf::Quaternion q(msg_ptr->pose.pose.orientation.x, msg_ptr->pose.pose.orientation.y, msg_ptr->pose.pose.orientation.z, msg_ptr->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        q.setRPY(roll, -pitch, -yaw);
        q.normalize();
        hil_state_quaternion_msg.attitude_quaternion[0] = q.getW();
        hil_state_quaternion_msg.attitude_quaternion[1] = q.getX();
        hil_state_quaternion_msg.attitude_quaternion[2] = q.getY();
        hil_state_quaternion_msg.attitude_quaternion[3] = q.getZ();
        hil_state_quaternion_msg.rollspeed  =  msg_ptr->velocity.twist.angular.x;
        hil_state_quaternion_msg.pitchspeed = -msg_ptr->velocity.twist.angular.y;
        hil_state_quaternion_msg.yawspeed   = -msg_ptr->velocity.twist.angular.z;
        msr::airlib::Vector3r position_local_ned;
        position_local_ned.x() =  msg_ptr->pose.pose.position.x;
        position_local_ned.y() = -msg_ptr->pose.pose.position.y;
        position_local_ned.z() = -msg_ptr->pose.pose.position.z;
        msr::airlib::EarthUtils::HomeGeoPoint home_geo(msr::airlib::GeoPoint(0, 0, 0));  // lat, lon, alt
        msr::airlib::GeoPoint geo_position = msr::airlib::EarthUtils::nedToGeodetic(position_local_ned, home_geo);
        hil_state_quaternion_msg.lat =  geo_position.latitude * 1e7;  // FIXME home gps should be changable
        hil_state_quaternion_msg.lon =  geo_position.longitude * 1e7;
        hil_state_quaternion_msg.alt =  geo_position.altitude * 1000;
        hil_state_quaternion_msg.vx  =  msg_ptr->velocity.twist.linear.x * 100;
        hil_state_quaternion_msg.vy  = -msg_ptr->velocity.twist.linear.y * 100;
        hil_state_quaternion_msg.vz  = -msg_ptr->velocity.twist.linear.z * 100;  // FIXME need to test
        double airspeed = sqrt(pow(msg_ptr->velocity.twist.linear.x, 2) + pow(msg_ptr->velocity.twist.linear.y, 2) + pow(msg_ptr->velocity.twist.linear.z, 2));
        hil_state_quaternion_msg.ind_airspeed  = airspeed * 100;
        hil_state_quaternion_msg.true_airspeed = airspeed * 100;
        hil_state_quaternion_msg.xacc =  msg_ptr->acceleration.accel.linear.x * 1e3;
        hil_state_quaternion_msg.yacc = -msg_ptr->acceleration.accel.linear.y * 1e3;
        hil_state_quaternion_msg.zacc = -msg_ptr->acceleration.accel.linear.z * 1e3;  // FIXME need to test
#if 0
        mavlink_message_t mavmsg;
        mavlink_msg_hil_state_quaternion_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_state_quaternion_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
#endif
    }
    
    void SensorImuCallback(const sensor_msgs::Imu::ConstPtr msg_ptr)
    {
        hil_sensor_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        // from NWU to NED
        hil_sensor_msg.xacc  =  msg_ptr->linear_acceleration.x;
        hil_sensor_msg.yacc  = -msg_ptr->linear_acceleration.y;
        hil_sensor_msg.zacc  = -msg_ptr->linear_acceleration.z;
        hil_sensor_msg.xgyro =  msg_ptr->angular_velocity.x;
        hil_sensor_msg.ygyro = -msg_ptr->angular_velocity.y;
        hil_sensor_msg.zgyro = -msg_ptr->angular_velocity.z;
        hil_sensor_msg.fields_updated = 0b111111;
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_sensor_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_sensor_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
    void SensorMagneticFieldCallback(const sensor_msgs::MagneticField::ConstPtr msg_ptr)
    {
        hil_sensor_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        // from Tesla to Gauss, 1T = 10000G
        hil_sensor_msg.xmag =  msg_ptr->magnetic_field.x * 10000;
        hil_sensor_msg.ymag = -msg_ptr->magnetic_field.y * 10000;
        hil_sensor_msg.zmag = -msg_ptr->magnetic_field.z * 10000;
        hil_sensor_msg.fields_updated = 0b111 << 6;
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_sensor_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_sensor_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
    void SensorAbsolutePressureCallback(const sensor_msgs::FluidPressure::ConstPtr msg_ptr)
    {
        hil_sensor_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        // from Pascals to Millibar, 1M = 100P
        hil_sensor_msg.abs_pressure = msg_ptr->fluid_pressure * 0.01;
        hil_sensor_msg.fields_updated = 0b1 << 9;
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_sensor_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_sensor_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
    void SensorDifferentialPressureCallback(const sensor_msgs::FluidPressure::ConstPtr msg_ptr)
    {
        hil_sensor_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        // from Pascals to Millibar, 1M = 100P
        hil_sensor_msg.diff_pressure = msg_ptr->fluid_pressure * 0.01;
        hil_sensor_msg.fields_updated = 0b1 << 10;
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_sensor_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_sensor_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
    void SensorPressureAltitudeCallback(const sensor_msgs::Range::ConstPtr msg_ptr)
    {
        hil_sensor_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        hil_sensor_msg.pressure_alt = msg_ptr->range;
        hil_sensor_msg.fields_updated = 0b1 << 11;
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_sensor_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_sensor_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
    void SensorTemperatureCallback(const sensor_msgs::Temperature::ConstPtr msg_ptr)
    {
        hil_sensor_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        hil_sensor_msg.temperature = msg_ptr->temperature;
        hil_sensor_msg.fields_updated = 0b1 << 12;
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_sensor_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_sensor_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
    void SensorGpsCallback(const sensor_msgs::NavSatFix::ConstPtr msg_ptr) const
    {
        mavlink_hil_gps_t hil_gps_msg;
        
        hil_gps_msg.time_usec = msg_ptr->header.stamp.toNSec() / 1000;
        hil_gps_msg.fix_type = msg_ptr->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX ? 3 : 0;
        hil_gps_msg.lat = msg_ptr->latitude * 1e7;
        hil_gps_msg.lon = msg_ptr->longitude * 1e7;
        hil_gps_msg.alt = msg_ptr->altitude * 1000;
        //hil_gps_msg.alt = (msg_ptr->altitude + mavutils::ellipsoid_to_geoid_height(msg_ptr)) * 1000;  // FIXME need to test
        
        hil_gps_msg.eph = 100;  // If unknown, set to: 65535
        hil_gps_msg.epv = 100;  // If unknown, set to: 65535
        hil_gps_msg.vel = 0;  // If unknown, set to: 65535
        
        // FIXME how to get GPS velocity?
        hil_gps_msg.vn = hil_state_quaternion_msg.vx;
        hil_gps_msg.ve = hil_state_quaternion_msg.vy;
        hil_gps_msg.vd = hil_state_quaternion_msg.vz;
        
        hil_gps_msg.cog = 65535;  // If unknown, set to: 65535
        hil_gps_msg.satellites_visible = 20;  // If unknown, set to: 255
        
        mavlink_message_t mavmsg;
        mavlink_msg_hil_gps_encode_chan(UAS_PACK_CHAN(uas_), &mavmsg, &hil_gps_msg);
        UAS_FCU(uas_)->send_message(&mavmsg);
    }
    
};  // class VehicleStatePlugin

};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::VehicleStatePlugin, mavplugin::MavRosPlugin)
