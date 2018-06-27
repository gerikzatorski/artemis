# artemis

## Packages

* artemis_description
* artemis_control
* artemis_gazebo

## Building

Navigate to the catkin workspace then

```sh
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Testing

### display in rviz

```
roslaunch artemis_description artemis_description.launch rviz:=true
```

### run control

```
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
rostopic pub -1 artemis/joint_position_controller/command std_msgs/Float64MultiArray '{ data: [0.0, 5.0] }'
```

## TODO

* CAD to urdf conversion
