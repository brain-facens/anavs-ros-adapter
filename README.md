# Anavs-ROS-adapter

## ROS package for the Anavs GNSS sensor

This package provides an adapter between ANavS TCP/IP data streams and ROS and allows to publish the ANavS sensor fusion solution (mode 'padsolution2ros') and ANavS sensor data (mode 'padsensordata2ros') in ROS.


## Installation

Given you have installed [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and set up your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), run the following commands:

```
cd ~/catkin_ws/src/
git clone https://github.com/brain-facens/anavs-ros-adapter.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ./devel/setup.bash
```

## Usage

### Anavs output format

To run the sensor and visualization on pre-configured Rviz file, run:

```
roslaunch anavs-ros-adapter anavs_display.launch
```

This will use the default IP address to `192.168.0.100`, to set a diffeerent IP address, use the argument `ip_addr:=value`

### Satellite fix

To get the output on the `sensor_msgs/NavSatFix` format, run:

```
roslaunch anavs-ros-adapter fix.launch
```

This will also publish the NMEA sentences on the `nmea_sentences` topic.
