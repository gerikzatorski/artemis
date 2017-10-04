# artemis

## Packages

* artemis_description
* artemis_control
* artemis_gazebo

## Testing

### display in rviz

```
roslaunch artemis_description artemis_description.launch
```

### run control

```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rostopic pub -1 artemis/joint_position_controller/command std_msgs/Float64MultiArray '{ data: [0.0, 5.0] }'
```

## TODO

* add ROS service for shooting
* CAD to urdf conversion
* Update ROS CMakeLists.txt and package.xml files
