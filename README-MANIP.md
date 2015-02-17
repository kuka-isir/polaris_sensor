## Flexibility analysis 

### Launch Polaris sensor

```bash
roscore
```

```bash
rosrun polaris_sensor polaris_sensor _port:=/dev/ttyUSB0 \
_roms:="$(rospack find polaris_sensor)/rom/end_effector.rom, \
	$(rospack find polaris_sensor)/rom/T0.rom, \
	$(rospack find polaris_sensor)/rom/T2.rom, \
	$(rospack find polaris_sensor)/rom/T3.rom, \
	$(rospack find polaris_sensor)/rom/T4.rom, \
	$(rospack find polaris_sensor)/rom/T5.rom, \
	$(rospack find polaris_sensor)/rom/T6.rom"
```

### Visualize
#### Cmd line
```bash
rostopic echo /polaris_sensor/targets
```

#### RviZ

##### (optional) Publish the polaris transform if not calibrated
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 /world /polaris_link 100
```


```bash
rosrun rviz rviz
# Then add a PointCloud message and listen to topic /polaris_sensor/targets_cloud
# Or add a PoseArray message and listen to /polaris_sensor/targets
```


#### Calibration
##### ToDo on the Kuka PC
* Turn ON the robot --> Joint values --> PC --> Orocos (RT) --> ROS
* Launch the lbr4_state_publisher --> /joint_states
* Launch the robot_state_publisher --> tf frames for all joints (/lbr4_1_link etc)
* Launch the tf_to_pointstamped --> target_kuka (tf frame) --> /kuka/tooltip (PointStamped)
##### ToDo on the Polaris PC
* Launch the Polaris sensor with the kuka tooltip target --> /polaris_sensor/targets (PointArrayStamped)
* Launch the polaris_listener --> /polaris_sensor/one_target (PointStamped)
##### Calibration Node on Polaris PC
* Launch compute_transform_6d --> Listens to /polaris_sensor/one_target and /kuka/tooltip and publishes the static transform beetween /base_link -> /polaris_link
