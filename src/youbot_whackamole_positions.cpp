#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#include <brics_actuator/JointPositions.h>
#include <sensor_msgs/JointState.h>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include <std_msgs/String.h>

//TODO: prevent simultaneous arm and base movement

#define PRINT_LASER_MSGS 0

#define LASER_ANGLE_THRESH_LEFT -0.8f
#define LASER_ANGLE_THRESH_FRONT 0.05f

float robot_dist_left_avg, robot_dist_front_avg, robot_angle_left_avg;
#define avg_merge 0.5f

// Desired position constants
//dist fronts: 2.1, 1.5, 0.9, 0.3
#define ROBOT_ANGLE_LEFT_DESIRED 0.0f
#define ROBOT_ANGLE_LEFT_SETTLE_THRESHOLD 0.03f
#define ROBOT_ANGLE_LEFT_ACCEPTABLE_RANGE 0.5f
#define ROBOT_ANGLE_LEFT_SETTLE_STEPS 4
#define ROBOT_DIST_LEFT_RANGE_MIN 0.10f
#define ROBOT_DIST_LEFT_RANGE_MAX 1.0f
#define ROBOT_DIST_LEFT_DESIRED 0.26f
#define ROBOT_DIST_LEFT_SETTLE_THRESHOLD 0.05f
#define ROBOT_DIST_LEFT_SETTLE_STEPS 4
#define ROBOT_DIST_FRONT_RANGE_MIN 0.15f
#define ROBOT_DIST_FRONT_RANGE_MAX 2.25f
#define ROBOT_DIST_FRONT_SETTLE_THRESHOLD 0.03f
#define ROBOT_DIST_FRONT_SETTLE_STEPS 5
#define ROBOT_ALL_SETTLE_STEPS 6

#define NUM_ROBOT_POSITIONS 4
float ROBOT_DIST_FRONT_POSITIONS[NUM_ROBOT_POSITIONS] = { 2.3f, 1.7f, 1.1f, 0.5f };
int desired_robot_position = -1;
int robot_position = -1;
//counters for being in the desired robot position
int robot_front_settled=0, robot_left_settled=0, robot_ang_settled=0;
int robot_all_settled=0;
ros::Time time_last_settled;

//absolute x and y velocity limits
#define ROBOT_MIN_Y_VELOCITY 0.1f
#define ROBOT_MAX_Y_VELOCITY 0.2f
#define ROBOT_Y_VELOCITY_KP 1.0f
#define ROBOT_MIN_X_VELOCITY 0.1f
#define ROBOT_MAX_X_VELOCITY 0.6f
#define ROBOT_X_VELOCITY_KP 1.0f
#define ROBOT_MIN_ANG_VELOCITY 0.1f
#define ROBOT_MAX_ANG_VELOCITY 0.3f
#define ROBOT_ANG_VELOCITY_KP 1.0f

// Function headers for arm
void initPosJoints();
bool isValidStartPosition();
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void armPositionCallback(const std_msgs::Int16::ConstPtr& armPos);
void pubArmPosCmd(int armPosCmd);
bool armCloseToGoal();

/// Publishes a message when a mole is whacked
ros::Publisher whack_impact_pub;
/// Publishes a message when a whack is complete
ros::Publisher whack_complete_pub;
/// Publishes a message when the robot has reached a position
ros::Publisher robot_position_arrive_pub;

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

//which position the arm is currently in
int arm_position = -1;
//which arm position to move the arm to
int desired_arm_position=-1;
//whether the arm should whack a mole when it reaches its desired position
bool should_whack=false;

#define ARM_JOINT_SETTLE_THRESHOLD 0.1

// Defines 5 joint angle values for each of the 7 predefined positions;
double positionJoints[NUM_ARM_POSITIONS][NUM_ARM_JOINTS];


//linearly interpolate two floats by a merge value.
//m=0 --> a
//m=0.5f --> halfway between a and b
//m=1 --> b
float lerp(float a, float b, float m)
{
  return (a*m+b*(1.0f-m));
}

//clamp the absolute value of float between two values
float aclamp(float minv, float val, float maxv)
{
  return fmax(minv,fmin(maxv,fabs(val)))*((val>0)?1.0f:-1.0f);
}

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
  //TODO: Implement this safety check
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

//determine if all arm joints are within the settle threshold
bool armCloseToGoal()
{
  if (desired_arm_position==-1) return true;
  if (desired_arm_position>=NUM_ARM_POSITIONS) return false;
  int i;
  for (i=0; i<NUM_ARM_JOINTS; i++)
  {
    if (fabs(currentJoint[i]-positionJoints[desired_arm_position][i])>ARM_JOINT_SETTLE_THRESHOLD) return false;
  }
  return true;
}

// Callback function to handle receiving of arm position commands
void armPositionCallback(const std_msgs::Int16::ConstPtr& armPos) {
  int ap = armPos->data;
  if (ap>=1 && ap<=3) {
    ROS_INFO("Whacking mole %d",ap-1+robot_position);
    desired_arm_position=ap*2-1; //1,3,5 to whack
    arm_position=ap;
    should_whack=true;
  }
  else ROS_WARN("Invalid arm position command: %d",ap);
}

void pubArmPosCmd(int armPosCmd) {
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

//receive position command
void robotPositionCallback(const std_msgs::Int16::ConstPtr& position)
{
  int p = position->data;
  ROS_INFO("Received robot position command: %d",p);
  if (p>=0 && p<NUM_ROBOT_POSITIONS) {
    desired_robot_position=p;
    robot_position=p;
  }
}

//receive init command. moves robot and arm to center positions
void robotInitCallback(const std_msgs::Empty::ConstPtr& position)
{
  desired_robot_position=2;
  robot_position=2;
  desired_arm_position=3; //1,3,5 to whack
  arm_position=2;
  should_whack=false;
}

//compute the distance and angle to left and front walls from laser scan
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (PRINT_LASER_MSGS) 
    ROS_INFO("laser scan: from %f to %f, increment %f",
            scan->angle_min, scan->angle_max, scan->angle_increment);

  int num_scans = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);

  geometry_msgs::Point points_left[num_scans];
  int num_left = 0;
  geometry_msgs::Point points_front[num_scans];
  int num_front = 0;
  geometry_msgs::Point p;

  float robot_dist_left=0, robot_dist_front=0;

  float angle;
  float i=0;
  for (angle = scan->angle_min; angle < scan->angle_max; angle+=scan->angle_increment)
  {
    //store points and compute the average dist from scans to the front
    if (angle>-LASER_ANGLE_THRESH_FRONT && angle<LASER_ANGLE_THRESH_FRONT)
    {
      p.x=scan->ranges[i]*cos(angle);
      p.y=scan->ranges[i]*-sin(angle);
      points_front[num_front++]=p;

      if (num_front==1) robot_dist_front=p.x;
      else robot_dist_front=(robot_dist_front+p.x)/2.0f;
    }
    //store points and compute the average dist from scans to the left
    else if (angle<LASER_ANGLE_THRESH_LEFT)
    {
      p.x=scan->ranges[i]*cos(angle);
      p.y=scan->ranges[i]*-sin(angle);
      //ignore points which might be 
      if (robot_dist_front_avg>0 && fabs(p.x)<robot_dist_front_avg) {
        points_left[num_left++]=p;

        if (num_left==1) robot_dist_left=p.y;
        else robot_dist_left=(robot_dist_left+p.y)/2.0f;
      }
      //else ROS_WARN("Point (%f,%f) too frontal (>%f)",p.x,p.y,robot_dist_front_avg);
    }

    i++;
  }

  if (PRINT_LASER_MSGS) ROS_INFO("    dist left: %f",robot_dist_left);
  if (PRINT_LASER_MSGS) ROS_INFO("    dist front: %f",robot_dist_front);
  robot_dist_left_avg=lerp(robot_dist_left_avg,robot_dist_left,avg_merge);
  robot_dist_front_avg=lerp(robot_dist_front_avg,robot_dist_front,avg_merge);

  //approximate line angles
  if (num_left>1)
  {
    float robot_angle_left;
    // TODO: use more than the first and last points to get the angle
    robot_angle_left=atan2(
        points_left[num_left-1].y-points_left[0].y,
        points_left[num_left-1].x-points_left[0].x
    );
    if (PRINT_LASER_MSGS) ROS_INFO("    angle left: %f  (from %d points)",robot_angle_left,num_left);
    robot_angle_left_avg=lerp(robot_angle_left_avg,robot_angle_left,avg_merge);
  }
  else ROS_WARN("   No points found to the left");

  if (num_front>1)
  {
    float angle_front;
    // TODO: use more than the first and last points to get the angle
    angle_front=atan2(
        points_front[num_front-1].y-points_front[0].y,
        points_front[num_front-1].x-points_front[0].x
    );
    if (PRINT_LASER_MSGS) ROS_INFO("    angle front: %f  (from %d points)",angle_front,num_front);
  }
  else ROS_WARN("   No points found to the front");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_whackamole_positions_node");
  ros::NodeHandle n;
  // Publishes robot command velocities
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  // Subscribes to laser scans
  ros::Subscriber laser_sub = n.subscribe("base_scan", 1000, laserCallback);
  // Subscribes to whack-a-mole robot position commands
  ros::Subscriber cmd_robot_pos_sub = n.subscribe("whackamole/cmd_robot_pos", 1000, robotPositionCallback);

  // Initialize robot position measurement averages
  robot_dist_left_avg=ROBOT_DIST_LEFT_DESIRED;
  robot_angle_left_avg=ROBOT_ANGLE_LEFT_DESIRED;
  robot_dist_front_avg=(ROBOT_DIST_FRONT_RANGE_MIN+ROBOT_DIST_FRONT_RANGE_MAX)/2.0f;

  // Publishes joint position commands
  arm_pos_pub = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  // Subscribes to whack-a-mole arm position commands
  ros::Subscriber arm_cmd_sub = n.subscribe("whackamole/cmd_arm_pos", 1000, armPositionCallback);
  // Subscribes to joint states
  ros::Subscriber joint_state_sub = n.subscribe("joint_states", 1000, jointStateCallback);

  ros::Subscriber init_sub = n.subscribe("whackamole/robot_init",1000,robotInitCallback);

  // Set the correct size for the arm positions vector
  armJointPositions.resize(NUM_ARM_JOINTS); //TODO:change that
  
  whack_impact_pub = n.advertise<std_msgs::Int16>("whackamole/whack_impact",1);
  whack_complete_pub = 
    n.advertise<std_msgs::Int16>("whackamole/whack_complete",1);
  robot_position_arrive_pub =
    n.advertise<std_msgs::Int16>("whackamole/robot_position_arrive",1);

  // Set up joint values for the various arm positions
  initPosJoints();
// Initialize joint states to zero, and wait for them to be filled in with actual values before proceeding.
  int i;
  for(i = 0; i < NUM_ARM_JOINTS; i++) {
    currentJoint[i] = -100;
  }

  // Now that the initial joint states have been recorded, check whether the arm
  // is in a valid and safe starting position
  if(!isValidStartPosition()) {
    ROS_ERROR("Arm is not in a valid start position.");
    return -1;
  }

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {

    //if trying to move to an arm position
    if (desired_arm_position>=0 && desired_arm_position<NUM_ARM_POSITIONS)
    {
      pubArmPosCmd(desired_arm_position);
      if (armCloseToGoal()) {
        //if moving to an odd position (whack-ready), whack
        if ((desired_arm_position%2)==1 && should_whack) desired_arm_position++;
        else if ((desired_arm_position%2)==0)
        {
          desired_arm_position--;
          should_whack=false;
          ROS_INFO("Whacked mole %d",
              (desired_arm_position/2+(robot_position-1)*2));
          std_msgs::Int16 msg;
          msg.data=(desired_arm_position/2+(robot_position-1)*2);
          whack_impact_pub.publish(msg);
        }
        else 
        {
          ROS_INFO("Done whacking mole %d",
              (desired_arm_position/2+(robot_position-1)*2));
          std_msgs::Int16 msg;
          msg.data=(desired_arm_position/2+(robot_position-1)*2);
          whack_complete_pub.publish(msg);
          desired_arm_position=-1;
        }
      }
    }//end if trying to move to an arm position
    else
    {
      desired_arm_position=-1;
      should_whack=false;
    }


    //if trying to move to a robot position,
    if (desired_robot_position>=0 && desired_robot_position<NUM_ROBOT_POSITIONS)
    {
      ROS_INFO("dleft: %fm,  dfront: %fm, aleft: %frad",
        robot_dist_left_avg, robot_dist_front_avg, robot_angle_left_avg);
      //check if outside acceptable positions
      bool in_range=true;
      if (robot_dist_left_avg<ROBOT_DIST_LEFT_RANGE_MIN)
      {
        in_range=false;
        ROS_WARN("Too close to wall on left. %f < %f", robot_dist_left_avg,ROBOT_DIST_LEFT_RANGE_MIN);
      }
      if (robot_dist_left_avg>ROBOT_DIST_LEFT_RANGE_MAX)
      {
        in_range=false;
        ROS_WARN("Too far from wall on left. %f > %f", robot_dist_left_avg,ROBOT_DIST_LEFT_RANGE_MAX);
      }
      if (robot_dist_front_avg<ROBOT_DIST_FRONT_RANGE_MIN)
      {
        in_range=false;
        ROS_WARN("Too close to wall on front. %f < %f", robot_dist_front_avg,ROBOT_DIST_FRONT_RANGE_MIN);
      }
      if (robot_dist_front_avg>ROBOT_DIST_FRONT_RANGE_MAX)
      {
        in_range=false;
        ROS_WARN("Too far from wall on front. %f < %f", robot_dist_front_avg,ROBOT_DIST_FRONT_RANGE_MAX);
      }
      if (fabs(robot_angle_left_avg-ROBOT_ANGLE_LEFT_DESIRED) > ROBOT_ANGLE_LEFT_ACCEPTABLE_RANGE)
      {
        in_range=false;
        ROS_WARN("Not parallel enough to wall. %f", robot_angle_left_avg);
      }

      if (!in_range) {
        ROS_WARN("Outside of valid robot positions, preventing movement");
        geometry_msgs::Twist v;
        v.linear.x=0;
        v.linear.y=0;
        v.angular.z=0;
        cmd_vel_pub.publish(v);
        desired_robot_position=-1;
      }
      else {
        //move to desired robot position
        //front distance down, linear x positive (negative error)
        //left distance down, linear y positive (negative error)
        //left angle down, angular z positive (negative error)
        geometry_msgs::Twist v;
        bool settled=true;
        //correct left distance
        float error = ROBOT_DIST_LEFT_DESIRED-robot_dist_left_avg;
        if (fabs(error)>ROBOT_DIST_LEFT_SETTLE_THRESHOLD)
        {
          settled=false;
          robot_left_settled=0;
          v.linear.y = aclamp(ROBOT_MIN_Y_VELOCITY, error*-ROBOT_Y_VELOCITY_KP, ROBOT_MAX_Y_VELOCITY);
        }
        else 
        {
          v.linear.y=0;
          robot_left_settled++;
          if (robot_left_settled < ROBOT_DIST_LEFT_SETTLE_STEPS) settled=false;
        }
        //correct front distance
        error = ROBOT_DIST_FRONT_POSITIONS[desired_robot_position]-robot_dist_front_avg;
        if (fabs(error)>ROBOT_DIST_FRONT_SETTLE_THRESHOLD)
        {
          settled=false;
          robot_front_settled=0;
          v.linear.x = aclamp(ROBOT_MIN_X_VELOCITY, error*-ROBOT_X_VELOCITY_KP, ROBOT_MAX_X_VELOCITY);
        }
        else 
        {
          v.linear.x=0;
          robot_front_settled++;
          if (robot_front_settled < ROBOT_DIST_FRONT_SETTLE_STEPS) settled=false;
        }
        //correct angle
        error = ROBOT_ANGLE_LEFT_DESIRED - robot_angle_left_avg;
        if (fabs(error)>ROBOT_ANGLE_LEFT_SETTLE_THRESHOLD)
        {
          settled=false;
          robot_ang_settled=0;
          v.angular.z = aclamp(ROBOT_MIN_ANG_VELOCITY, error*-ROBOT_ANG_VELOCITY_KP, ROBOT_MAX_ANG_VELOCITY);
        }
        else 
        {
          v.angular.z=0;
          robot_ang_settled++;
          if (robot_ang_settled < ROBOT_ANGLE_LEFT_SETTLE_STEPS) settled=false;
        }

        cmd_vel_pub.publish(v);

        if (settled) 
        {
          robot_all_settled++;
          if (robot_all_settled>=ROBOT_ALL_SETTLE_STEPS)
          {
            ROS_INFO("\n\nGot to position %d in %f secs\n",desired_robot_position,(ros::Time::now()-time_last_settled).toSec());
            std_msgs::Int16 msg;
            msg.data=desired_robot_position;
            robot_position_arrive_pub.publish(msg);
            desired_robot_position=-1;
          }
        }
        else robot_all_settled=0;
      } //end if in range and going to position

    } //end if going to robot position
    else
    {
      robot_left_settled=0;
      robot_front_settled=0;
      robot_ang_settled=0;
      robot_all_settled=0;
      desired_robot_position=-1;
      time_last_settled=ros::Time::now();
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

}
