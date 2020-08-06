# Overview

## What does this software do?

It's forked from official mavros version. Some bugs has been fixed and some functions has been added such as multi UAV and HITL support.

## Install
```sudo apt-get install libgeographic14```

```sudo apt-get install ros-kinetic-mavlink```

# Additional ROS API

## mavros (vehicle state plugin)

### Subscribed Topics

- */state/measurement (nus_msgs::StateWithCovarianceStamped)*
Estimated UAV measurement state, which will be sent to PX4.
- */state/reference (nus_msgs::StateWithCovarianceStamped)*
UAV reference state, which will be sent to PX4.
- */hil/state/ground_truth (nus_msgs::StateWithCovarianceStamped)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL ground truth data, will be sent to PX4.
- */hil/sensor/imu (sensor_msgs::Imu)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL imu data, will be sent to PX4.
- */hil/sensor/magnetic_field (sensor_msgs::MagneticField)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL magnetic field data, will be sent to PX4.
- */hil/sensor/absolute_pressure (sensor_msgs::FluidPressure)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL absolute presssure data, will be sent to PX4.
- */hil/sensor/differential_pressure (sensor_msgs::FluidPressure)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL differential pressure data, will be sent to PX4.
- */hil/sensor/pressure_altitude (sensor_msgs::Range)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL pressure altitude data, will be sent to PX4.
- */hil/sensor/temperature (sensor_msgs::Temperature)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL temperature data, will be sent to PX4.
- */hil/sensor/gps (sensor_msgs::NavSatFix)*
Valid if param *~/vehicle_state/enable_hil* is true. HITL GPS data, will be sent to PX4.

### Parameters

- *~/vehicle_state/enable_hil* (bool, default: false)
If enabled, HITL related messages will be sent to PX4.

MAVROS
======

MAVLink extendable communication node for ROS.

Since 2014-08-11 this repository contains several packages.


mavros package
--------------

It is the main package, please see it's [README][mrrm].


mavros\_extras package
----------------------

This package contain some extra nodes and plugins for mavros, please see it's [README][exrm].


libmavconn package
------------------

This package contain mavconn library, see it's [README][libmc].
MAVConn may be used outside of ROS environment.


CI Statuses
-----------

  - ROS Hydro: [![Hydro build status](http://jenkins.ros.org/buildStatus/icon?job=devel-hydro-mavros)](http://jenkins.ros.org/job/devel-hydro-mavros/)
  - ROS Indigo: [![Indigo build status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-mavros)](http://jenkins.ros.org/job/devel-indigo-mavros/)
  - Travis Hydro (PX4): [![Hydro px4 status](https://travis-ci.org/mavlink/mavros.svg?branch=master)](https://travis-ci.org/mavlink/mavros)
  - Travis Hydro (Coverity Scan): [![Hydro scan status](https://travis-ci.org/mavlink/mavros.svg?branch=coverity_scan)](https://travis-ci.org/mavlink/mavros)


[mrrm]: https://github.com/mavlink/mavros/blob/master/mavros/README.md
[exrm]: https://github.com/mavlink/mavros/blob/master/mavros_extras/README.md
[libmc]: https://github.com/mavlink/mavros/blob/master/libmavconn/README.md
