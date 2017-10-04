/*
 * This is the code running on the arduino attached to Artemis.
 */

// reverse compatability
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

// ROS
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

// actuator libraries
#include <AccelStepper.h>
#include "SimpleRelay.h"

#define STEPPER_PAN_DIR 4    // Y
#define STEPPER_PAN_STEP 5   // Y
#define STEPPER_TILT_DIR 7   // X
#define STEPPER_TILT_STEP 6  // X

#define RELAY_FLYWHEELS_PIN 10
#define RELAY_BELT_PIN 11

// Function Prototypes
void artemis_cb(const std_msgs::Float64MultiArray& cmd_msg);
void artemis_fire(const std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

// zero position is set at construction time
AccelStepper stepper_pan(AccelStepper::DRIVER, STEPPER_PAN_STEP, STEPPER_PAN_DIR);
AccelStepper stepper_tilt(AccelStepper::DRIVER, STEPPER_TILT_STEP, STEPPER_TILT_DIR);

SimpleRelay relay_flywheel(RELAY_FLYWHEELS_PIN);
SimpleRelay relay_belt(RELAY_BELT_PIN);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64MultiArray> sub("/artemis/joint_position_controller/command", &artemis_cb);
ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse> service("/artemis/blaster/command", &artemis_fire);

void setup()
{
  pinMode(13, OUTPUT); // Arduino built-in LED

  // ROS setup stuff
  nh.initNode();
  nh.subscribe(sub);
  nh.advertiseService(service);

  // configure stepper motors
  stepper_pan.setMaxSpeed(30.0);
  stepper_pan.setAcceleration(5.0);
  stepper_tilt.setMaxSpeed(100.0);
  stepper_tilt.setAcceleration(40.0);
}

void loop()
{
  /* Debugging */
  /* nh.loginfo(info); */

  nh.spinOnce();
  delay(5);
}

/**************************************************
 *                      HELPERS
 **************************************************/
void artemis_cb(const std_msgs::Float64MultiArray& cmd_msg)
{
  long pan = cmd_msg.data[0];
  long tilt = cmd_msg.data[1];

  stepper_pan.moveTo(pan);
  stepper_tilt.moveTo(tilt);

  stepper_tilt.run();
  stepper_pan.run();
}

void artemis_fire(const std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{

  nh.loginfo("Firing Artemis!");
  
  /* start flywheel */
  relay_flywheel.NO();
  delay(300);
    
  relay_belt.NO();
  delay(300);
  relay_belt.NC();

  delay(100);
  relay_flywheel.NO();

  response.success = true;
  response.message = "0";
}
