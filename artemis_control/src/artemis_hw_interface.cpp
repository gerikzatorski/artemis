/*
 * Filename: artemis_hw_interface.cpp
 * Author: Gerik Zatorski
 */

#include <artemis_control/artemis_hw_interface.h>

namespace artemis_control
{

  ArtemisHWInterface::ArtemisHWInterface(ros::NodeHandle nh)
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_j1("joint1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_j1);
    hardware_interface::JointStateHandle state_handle_j2("joint2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_j2);
    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_j1(jnt_state_interface.getHandle("joint1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_j1);
    hardware_interface::JointHandle pos_handle_j2(jnt_state_interface.getHandle("joint2"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_j2);
    registerInterface(&jnt_pos_interface);
  }

  bool ArtemisHWInterface::init()
  {
    // Start Command Publisher
    pub_joint_command = nh.advertise<std_msgs::Float64MultiArray>("/artemis/joint_position/controller/command", 10);
  }
  
  void ArtemisHWInterface::read(ros::Duration &elapsed_time)
  {}

  void ArtemisHWInterface::write(ros::Duration &elapsed_time)
  {
    /* set internal state */
    pantilt_msg.data[0] = cmd[0];
    pantilt_msg.data[1] = cmd[1];

    /* publish pan/tilt values */
    pub_joint_command.publish(pantilt_msg);
  }

} // namespace
