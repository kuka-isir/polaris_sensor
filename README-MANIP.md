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

##### (optional) Publish the polaris transform if not provided
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 /world /polaris_link 100
```


```bash
rosrun rviz rviz
# Then add a PointCloud message and listen to topic /polaris_sensor/targets_cloud
# Or add a PoseArray message and listen to /polaris_sensor/targets
```



