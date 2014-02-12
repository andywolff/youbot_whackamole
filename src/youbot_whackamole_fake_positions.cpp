#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <math.h>


#include <iostream>
#include <vector>
#include <stdio.h>
#include <unistd.h>

#include <std_msgs/String.h>

#define NUM_ROBOT_POSITIONS 4
int desired_robot_position = -1;
int robot_position = -1;
int last_robot_position = -1;
ros::Time robot_move_time_start;
ros::Duration robot_move_duration_short(0.5);
ros::Duration robot_move_duration_long(0.8);
ros::Time robot_move_time_end;

void armPositionCallback(const std_msgs::Int16::ConstPtr& armPos);
void pubArmPosCmd(int armPosCmd);

/// Publishes a message when a mole is whacked
ros::Publisher whack_impact_pub;
/// Publishes a message when a whack is complete
ros::Publisher whack_complete_pub;
/// Publishes a message when the robot has reached a position
ros::Publisher robot_position_arrive_pub;

// Publishes joint position commands
ros::Publisher arm_pos_pub;

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
int last_arm_position;
//whether the arm should whack a mole when it reaches its desired position
bool should_whack=false;

ros::Time arm_move_time_start;
ros::Duration arm_move_duration_short(0.5);
ros::Duration arm_move_duration_long(0.9);
ros::Duration arm_whack_duration(0.2);
ros::Time arm_move_time_end;

//linearly interpolate two floats by a merge value.
//m=0 --> a
//m=0.5f --> halfway between a and b
//m=1 --> b
float lerp(float a, float b, float m)
{
  return (a*m+b*(1.0f-m));
}

//takes a merge value between 0 and 1
//and smooths it like a slight s-curve
float smoothstep(float m) {
  return m*m*(3-2*m);
}

//clamp the absolute value of float between two values
float aclamp(float minv, float val, float maxv)
{
  return fmax(minv,fmin(maxv,fabs(val)))*((val>0)?1.0f:-1.0f);
}

// Callback function to handle receiving of arm position commands
void armPositionCallback(const std_msgs::Int16::ConstPtr& armPos) {
  int ap = armPos->data;
  if (ap>=1 && ap<=3) {
  
    desired_arm_position=ap*2-1; //1,3,5 to whack
  
    arm_move_time_start=ros::Time::now();
    if (abs(desired_arm_position-arm_position)==2)
      arm_move_time_end=arm_move_time_start+arm_move_duration_short;
    else
      arm_move_time_end=arm_move_time_start+arm_move_duration_long;
  
    ROS_INFO("Whacking mole %d",ap-1+robot_position);
    
    arm_position=ap;
    should_whack=true;
  }
  else ROS_WARN("Invalid arm position command: %d",ap);
}

//receive position command
void robotPositionCallback(const std_msgs::Int16::ConstPtr& position)
{
  int p = position->data;
  ROS_INFO("Received robot position command: %d",p);
  if (p>=0 && p<NUM_ROBOT_POSITIONS) {
    robot_move_time_start=ros::Time::now();
    if (p==robot_position)
      robot_move_time_end=robot_move_time_start;
    else if (abs(p-robot_position)<=1)
      robot_move_time_end=robot_move_time_start+robot_move_duration_short;
    else 
      robot_move_time_end=robot_move_time_start+robot_move_duration_long;
    desired_robot_position=p;
    robot_position=p;
  }
}

//receive init command. moves robot and arm to center positions
void robotInitCallback(const std_msgs::Empty::ConstPtr& position)
{
  robot_move_time_start=ros::Time::now();
  robot_move_time_end=robot_move_time_start;
  arm_move_time_start=robot_move_time_start;
  arm_move_time_end=robot_move_time_start;
  
  last_robot_position=robot_position=desired_robot_position=2; //mid
  last_arm_position=arm_position=desired_arm_position=3; //2,4,6 = whack
  should_whack=false;
}


int armCloseToGoal() {
  return ros::Time::now()>=arm_move_time_end;
}
int robotCloseToGoal() {
  return ros::Time::now()>=robot_move_time_end;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_whackamole_fake_positions_node");
  ros::NodeHandle n;
  
  // Subscribes to whack-a-mole robot position commands
  ros::Subscriber cmd_robot_pos_sub = n.subscribe("whackamole/cmd_robot_pos", 1000, robotPositionCallback);

  // Subscribes to whack-a-mole arm position commands
  ros::Subscriber arm_cmd_sub = n.subscribe("whackamole/cmd_arm_pos", 1000, armPositionCallback);

  ros::Subscriber init_sub = n.subscribe("whackamole/robot_init",1000,robotInitCallback);
  
  whack_impact_pub = n.advertise<std_msgs::Int16>("whackamole/whack_impact",1);
  whack_complete_pub = 
    n.advertise<std_msgs::Int16>("whackamole/whack_complete",1);
  robot_position_arrive_pub =
    n.advertise<std_msgs::Int16>("whackamole/robot_position_arrive",1);
    
    robot_move_time_start=ros::Time::now();
    robot_move_time_end=robot_move_time_start;
    arm_move_time_start=robot_move_time_start;
    arm_move_time_end=robot_move_time_start;

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {

    //if trying to move to an arm position
    if (desired_arm_position>=0 && desired_arm_position<NUM_ARM_POSITIONS)
    {
      if (armCloseToGoal()) {
        last_arm_position=desired_arm_position;
        //if moving to an odd position (whack-ready), whack
        if ((desired_arm_position%2)==1 && should_whack) 
        {
          desired_arm_position++;
          arm_move_time_start=ros::Time::now();
          arm_move_time_end=arm_move_time_start+arm_whack_duration;
        }
        else if ((desired_arm_position%2)==0)
        {
          arm_move_time_start=ros::Time::now();
          arm_move_time_end=arm_move_time_start+arm_whack_duration;
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
      
      if (robotCloseToGoal()) 
      {
        last_robot_position=desired_robot_position;
        //ROS_INFO("\n\nGot to position %d in %f secs\n",desired_robot_position,(ros::Time::now()-robot_move_time_start).toSec());
        std_msgs::Int16 msg;
        msg.data=desired_robot_position;
        robot_position_arrive_pub.publish(msg);
        desired_robot_position=-1;
      }

    } //end if going to robot position
    else
    {
      desired_robot_position=-1;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

}
