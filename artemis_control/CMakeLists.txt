cmake_minimum_required(VERSION 2.8.3)
project(artemis_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    controller_manager
)
