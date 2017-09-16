# artemis

## artemis_description

```
roslaunch artemis_description artemis_rviz.launch
```

## artemis_control


## artemis_gazebo

Artemis gazebo sim testing:

```
roslaunch artemis_gazebo artemis_world.launch
roslaunch artemis_control artemis_control.launch
rostopic pub -1 /artemis/joint2_position_controller/command std_msgs/Float64 "data: 100"
```

## TODO

* Update ROS CMakeLists.txt and package.xml files
