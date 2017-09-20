cmake_minimum_required(VERSION 2.8.3)
project(artemis_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    controller_manager
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(artemis_control
  src/artemis_hw_interface.cpp
)
target_link_libraries(artemis_control
  ${catkin_LIBRARIES}
)

add_executable(artemis_node
  src/artemis_node.cpp
)
target_link_libraries(artemis_node
  artemis_control
  ${catkin_LIBRARIES}
)
