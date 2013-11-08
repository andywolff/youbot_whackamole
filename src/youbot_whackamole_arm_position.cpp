#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include "std_msgs/String.h"

// Function headers
void initPosJoints();
bool isValidStartPosition();
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void armPositionCallback(const std_msgs::Int16::ConstPtr& armPos);
void moveArmToPos(int armPosCmd);

// Publishes joint position commands
ros::Publisher arm_pos_pub;
// Vector to hold arm joint position command message data
std::vector <brics_actuator::JointValue> armJointPositions;

// Constant defining the number of joints in arm
#define NUM_ARM_JOINTS 5

// Holds the most recent set of arm joint states
double currentJoint[NUM_ARM_JOINTS];

// Min/max joint angles for safety in case positions are given invalid joint positions
double jointMax[] = {5.840139, 2.617989, -0.0157081, 3.42919, 5.641589};
double jointMin[] = {0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062};

// Define arm positions
#define NUM_ARM_POSITIONS 7
// The position that the robot goes to on startup.
#define BASE_POS 0
#define LEFT_UP 1
#define LEFT_WHACK 2
#define CENTER_UP 3
#define CENTER_WHACK 4
#define RIGHT_UP 5
#define RIGHT_WHACK 6
// Defines 5 joint angle values for each of the 7 predefined positions;
double positionJoints[NUM_ARM_POSITIONS][NUM_ARM_JOINTS];

void initPosJoints() {
  positionJoints[BASE_POS][0] = 0.0100692;
  positionJoints[BASE_POS][1] = 0.0100692;
  positionJoints[BASE_POS][2] = -0.0157081;
  positionJoints[BASE_POS][3] = 0.0221391;
  positionJoints[BASE_POS][4] = 0.11062;

  positionJoints[LEFT_UP][0] = 3.74131;
  positionJoints[LEFT_UP][1] = 0.270872;
  positionJoints[LEFT_UP][2] = -0.216126;
  positionJoints[LEFT_UP][3] = 0.567257;
  positionJoints[LEFT_UP][4] = 0.11062;

  positionJoints[LEFT_WHACK][0] = 3.74131;
  positionJoints[LEFT_WHACK][1] = 0.270872;
  positionJoints[LEFT_WHACK][2] = -1.21821;
  positionJoints[LEFT_WHACK][3] = 0.0221472;
  positionJoints[LEFT_WHACK][4] = 0.11062;

  positionJoints[CENTER_UP][0] = 4.44092;
  positionJoints[CENTER_UP][1] = 0.0100692;
  positionJoints[CENTER_UP][2] = -0.917622;
  positionJoints[CENTER_UP][3] = 1.5894;
  positionJoints[CENTER_UP][4] = 0.11062;

  positionJoints[CENTER_WHACK][0] = 4.44092;
  positionJoints[CENTER_WHACK][1] = 0.0100692;
  positionJoints[CENTER_WHACK][2] = -2.32062;
  positionJoints[CENTER_WHACK][3] = 1.5894;
  positionJoints[CENTER_WHACK][4] = 0.11062;

  positionJoints[RIGHT_UP][0] = 5.14053;
  positionJoints[RIGHT_UP][1] = 0.270872;
  positionJoints[RIGHT_UP][2] = -0.216126;
  positionJoints[RIGHT_UP][3] = 0.567257;
  positionJoints[RIGHT_UP][4] = 0.11062;

  positionJoints[RIGHT_WHACK][0] = 5.14053;
  positionJoints[RIGHT_WHACK][1] = 0.270872;
  positionJoints[RIGHT_WHACK][2] = -1.21825;
  positionJoints[RIGHT_WHACK][3] = 0.0902801;
  positionJoints[RIGHT_WHACK][4] = 0.11062;

/*
  positionJoints[BASE_POS] = {0.0100692, 0.0100692, -0.0157081, 0.0221391, 0.11062};
  positionJoints[LEFT_UP] = {3.74131, 0.270872, -0.216126, 0.567257, 0.11062};
  positionJoints[LEFT_WHACK] = {3.74131, 0.270872, -1.21821, 0.0221472, 0.11062};
  positionJoints[CENTER_UP] = {4.44092, 0.0100692, -0.917622, 1.5894, 0.11062};
  positionJoints[CENTER_WHACK] = {4.44092, 0.0100692, -2.32062, 1.5894, 0.11062};
  positionJoints[RIGHT_UP] = {5.14053, 0.270872, -0.216126, 0.567257, 0.11062};
  positionJoints[RIGHT_WHACK] = {5.14053, 0.270872, -1.21825, 0.0902801, 0.11062};
*/

}

// Verifies that the robot is in a valid starting state
bool isValidStartPosition() {
	return true;
}

// Callback function to handle receiving of joint state messages
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if(msg->name[0] == "arm_joint_1") {
    currentJoint[0] = msg->position[0];
    currentJoint[1] = msg->position[1];
    currentJoint[2] = msg->position[2];
    currentJoint[3] = msg->position[3];
    currentJoint[4] = msg->position[4];
  }
} 

// Callback function to handle receiving of arm position commands
void armPositionCallback(const std_msgs::Int16::ConstPtr& armPos) {
  int ap = armPos->data;
  ROS_INFO("Received arm position command: %d",ap);
  if (ap>=0 && ap<NUM_ARM_POSITIONS) {
    ROS_INFO("Will move arm to position %d",ap);
    moveArmToPos(ap);
  }
}

void moveArmToPos(int armPosCmd) {
  // String for storing the name for each joint, used in
  // position messages
  std::stringstream jointName;

  // Construct the position message data for each joint
  for(int i = 0; i < NUM_ARM_JOINTS; i++)
  {
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);

    // Checking that the position does not give invalid joint values
    if(positionJoints[armPosCmd][i] < jointMin[i]) {
      positionJoints[armPosCmd][i] = jointMin[i];
    }
    if(positionJoints[armPosCmd][i] > jointMax[i]) {
      positionJoints[armPosCmd][i] = jointMax[i];
    }

    armJointPositions[i].joint_uri = jointName.str();
    armJointPositions[i].value = positionJoints[armPosCmd][i];

    armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }

  // Construct the joint position command to send
  brics_actuator::JointPositions command;
  command.positions = armJointPositions;

  // Send the joint position command
  arm_pos_pub.publish(command);
}

// Main function for running whackamole arm position node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_whackamole_arm_position_node");
  ros::NodeHandle n;

  // Publishes joint position commands
  arm_pos_pub = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  // Subscribes to whack-a-mole arm position commands
  ros::Subscriber arm_cmd_sub = n.subscribe("whackamole/cmd_arm_pos", 1000, armPositionCallback);
  // Subscribes to joint states
  ros::Subscriber joint_state_sub = n.subscribe("joint_states", 1000, jointStateCallback);
  // Sets the wait time between each iteration of the main loop
  ros::Rate loop_rate(10.0);

  // Set the correct size for the arm positions vector
  armJointPositions.resize(NUM_ARM_JOINTS); //TODO:change that

  // Set up joint values for the various arm positions
  initPosJoints();

  // Initialize joint states to zero, and wait for them to be filled in with actual values before proceeding.
  int i;
  for(i = 0; i < NUM_ARM_JOINTS; i++) {
    currentJoint[i] = -100;
  }
  /*while(currentJoint[3] == -100) {
    loop_rate.sleep();
    ros::spinOnce();
    ROS_INFO("BAD");
  }*/

  // Now that the initial joint states have been recorded, check whether the arm
  // is in a valid and safe starting position
  if(!isValidStartPosition()) {
    ROS_ERROR("Arm is not in a valid start position.");
    return -1;
  }

  // The main loop
  while (ros::ok())
  {
    
    loop_rate.sleep();
    ros::spinOnce();
  }

}
