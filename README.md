# artemis

## artemis_description

```
roslaunch artemis_description artemis_description.launch
```

## artemis_control

```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rostopic pub -1 artemis/joint_position_controller/command std_msgs/Float64MultiArray '{ data: [0.0, 5.0] }'
```

## artemis_gazebo

## TODO

* add ROS service for shooting
* CAD to urdf conversion
* Update ROS CMakeLists.txt and package.xml files
