# MAVROS for 168

## Install

- install dependency:

```
sudo apt install libgeographic14
sudo apt install ros-melodic-mavlink
```

- init serial port:

```
sudo su  # root permission
bash ./init_serial.sh
```

- mavros&mavlink

copy the files under mavlink folder to "/opt/ros/melodic/include/mavlink" overwirte existing files.

copy the files under mavros folder to "~/catkin_ws/src/" and compile.

remember to source setup.bash in catkin_ws/devel.

## Usage

roslaunch mavros px4.launch


##Authors & Contact
Gavin: [Xue.JW@qq.com](mailto:Xue.JW@qq.com)
