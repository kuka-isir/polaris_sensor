Polaris Hybrid Position Sensor driver
==============
This package contains a ROS-independant library to get information from the Polaris (based on https://github.com/wjwwood/serial) and a simple ROS wrapper (using catkin) to send a geometry_msgs::PoseArray (visualizable on rviz).

### Usage
Two parameters are needed, the .rom files and the port to which the sensor is connected :
```bash
rosrun polaris_sensor polaris_sensor _roms:=/home/T0.rom _port:=/dev/ttyUSB0
```

If you have **multiple** rom files :  
```bash
rosrun polaris_sensor polaris_sensor _roms:="$(rospack find polaris_sensor)/rom/kuka.rom,$(rospack find polaris_sensor)/rom/T0.rom" _port:=/dev/ttyUSB0
```

>Note: The rate is 60Hz.


> Authors : Antoine Hoarau, Florian Richer
