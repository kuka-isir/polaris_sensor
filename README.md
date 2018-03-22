Polaris Hybrid Position Sensor driver
==============
This package contains a ROS-independant library to get information from the Polaris (based on https://github.com/wjwwood/serial) and a simple ROS wrapper (using catkin) to send a geometry_msgs::PoseArray (visualizable on rviz).

##### Build status
[![Build Status](https://travis-ci.org/kuka-isir/polaris_sensor.svg?branch=master)](https://travis-ci.org/kuka-isir/polaris_sensor)

###### forked package, change the Cmakelist.txt

If you plan to access the Polaris Vicra from a non-root user account, you will need to add the

user account to the “lock” group. It may also be necessary to add the user account to the

following groups:

uucp

tty

dialout

# Please run:
'sudo usermod -G group_name account_name' for the above three groups

# To run the node for mutiple rom files:
'rosrun polaris_sensor polaris_sensor_node _roms:="$(rospack find polaris_sensor)/rom/kuka.rom,$(rospack find polaris_sensor)/rom/T0.rom" _port:=/dev/ttyUSB0'

The marker files(.rom) are in the 'rom' folder

>Note: The rate is 60Hz, supposed to be the max rate.
