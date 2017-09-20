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

// actuator libraries
#include <AccelStepper.h>
#include "SimpleRelay.h"

#define STEPPER_PAN_DIR 4    // Y
#define STEPPER_PAN_STEP 5   // Y
#define STEPPER_TILT_DIR 7   // X
#define STEPPER_TILT_STEP 6  // X

#define RELAY_FLYWHEELS_PIN 10
#define RELAY_BELT_PIN 11

// zero position is set at construction time
AccelStepper stepper_pan(AccelStepper::DRIVER, STEPPER_PAN_STEP, STEPPER_PAN_DIR);
AccelStepper stepper_tilt(AccelStepper::DRIVER, STEPPER_TILT_STEP, STEPPER_TILT_DIR);

SimpleRelay relay_flywheel(RELAY_FLYWHEELS_PIN);
SimpleRelay relay_belt(RELAY_BELT_PIN);

ros::NodeHandle nh;

// Function Prototypes
void artemis_cb(const std_msgs::Float64MultiArray& cmd_msg);

// Subscribe to Pan/Tilt Position Commands
ros::Subscriber<std_msgs::Float64MultiArray> sub("/artemis/joint_position_controller/command", &artemis_cb);

void setup()
{
  pinMode(13, OUTPUT); // Arduino built-in LED

  // ROS setup stuff
  nh.initNode();
  nh.subscribe(sub);

  // configure stepper motors
  stepper_pan.setMaxSpeed(30.0);
  stepper_pan.setAcceleration(5.0);
  stepper_tilt.setMaxSpeed(100.0);
  stepper_tilt.setAcceleration(40.0);
}

void loop()
{
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
  digitalWrite(13, HIGH-digitalRead(13));  // toggle led  
}
