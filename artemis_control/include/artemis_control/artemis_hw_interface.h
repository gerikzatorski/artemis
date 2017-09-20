/*
  Filename: artemis_hw_interface.h
  Author: Gerik Zatorski
*/

#ifndef ARTEMIS_HW_INTERFACE_H
#define ARTEMIS_HW_INTERFACE_H

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64MultiArray.h>

namespace artemis_control
{

  class ArtemisHWInterface : public hardware_interface::RobotHW
  {

  public:

    /// \brief Constructor
    ArtemisHWInterface(ros::NodeHandle nh);

    /// \brief Destructor
    ~ArtemisHWInterface() {}

    /// \brief Initialize Artemis robot
    bool init();
    
    /** \brief Read the state from the robot hardware. */
    virtual void read(ros::Duration &elapsed_time);

    /** \brief Write the command to the robot hardware. */
    virtual void write(ros::Duration &elapsed_time);

  private:
    ros::NodeHandle nh;
    
    // ROS Interfaces
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    // ROS Publishers
    ros::Publisher pub_joint_command;

    // ROS Messages
    std_msgs::Float64MultiArray pantilt_msg;

    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

  }; // class

} // namespace

#endif // ARTEMIS_HW_INTERFACE
