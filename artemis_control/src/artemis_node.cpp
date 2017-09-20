#include <artemis_control/artemis_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "artemis_node");
  ros::NodeHandle nh;
  
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(1);
  spinner.start();

  artemis_control::ArtemisHWInterface artemis(nh);

  ros::spin();

  return 0;
}
