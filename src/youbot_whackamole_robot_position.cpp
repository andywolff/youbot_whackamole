#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define PRINT_LASER_MSGS 0

#define LASER_ANGLE_THRESH_LEFT -0.8f
#define LASER_ANGLE_THRESH_FRONT 0.05f

float dist_left_avg, dist_front_avg, angle_left_avg;
#define avg_merge 0.4f

// Desired position constants
//dist fronts: 2.1, 1.5, 0.9, 0.3
#define ANGLE_LEFT_DESIRED 0.0f
#define ANGLE_LEFT_SETTLE_THRESHOLD 0.05f
#define ANGLE_LEFT_ACCEPTABLE_RANGE 0.3f
#define DIST_LEFT_RANGE_MIN 0.10f
#define DIST_LEFT_RANGE_MAX 0.5f
#define DIST_LEFT_DESIRED 0.27f
#define DIST_LEFT_SETTLE_THRESHOLD 0.05f
#define DIST_FRONT_RANGE_MIN 0.15f
#define DIST_FRONT_RANGE_MAX 2.25f
#define DIST_FRONT_SETTLE_THRESHOLD 0.05f

#define NUM_ROBOT_POSITIONS 4
float DIST_FRONT_POSITIONS[NUM_ROBOT_POSITIONS] = { 2.1f, 1.5f, 0.9f, 0.3f };
int desired_position = -1;

//absolute x and y velocity limits
#define MIN_Y_VELOCITY 0.05f
#define MAX_Y_VELOCITY 0.2f
#define Y_VELOCITY_KP 1.0f
#define MIN_X_VELOCITY 0.1f
#define MAX_X_VELOCITY 0.3f
#define X_VELOCITY_KP 1.0f
#define MIN_ANG_VELOCITY 0.05f
#define MAX_ANG_VELOCITY 0.2f
#define ANG_VELOCITY_KP 1.0f

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

//receive position command
void robotPositionCallback(const std_msgs::Int16::ConstPtr& position)
{
  int p = position->data;
  ROS_INFO("Received robot position command: %d",p);
  if (p>=0 && p<NUM_ROBOT_POSITIONS) desired_position=p;
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

  float dist_left=0, dist_front=0;

  float angle;
  float i=0;
  for (angle = scan->angle_min; angle < scan->angle_max; angle+=scan->angle_increment)
  {
    //store points and compute the average dist from scans to the left
    if (angle<LASER_ANGLE_THRESH_LEFT)
    {
      p.x=scan->ranges[i]*cos(angle);
      p.y=scan->ranges[i]*sin(angle);
      points_left[num_left++]=p;

      if (num_left==1) dist_left=p.x;
      else dist_left=(dist_left+p.x)/2.0f;
    }
    //store points and compute the average dist from scans to the front
    else if (angle>-LASER_ANGLE_THRESH_FRONT && angle<LASER_ANGLE_THRESH_FRONT)
    {
      p.x=scan->ranges[i]*cos(angle);
      p.y=scan->ranges[i]*sin(angle);
      points_front[num_front++]=p;

      if (num_front==1) dist_front=p.x;
      else dist_front=(dist_front+p.x)/2.0f;
    }

    i++;
  }

  if (PRINT_LASER_MSGS) ROS_INFO("    dist left: %f",dist_left);
  if (PRINT_LASER_MSGS) ROS_INFO("    dist front: %f",dist_front);
  dist_left_avg=lerp(dist_left_avg,dist_left,avg_merge);
  dist_front_avg=lerp(dist_front_avg,dist_front,avg_merge);

  //approximate line angles
  if (num_left>1)
  {
    float angle_left;
    // TODO: use more than the first and last points to get the angle
    angle_left=atan2(
        points_left[num_left-1].y-points_left[0].y,
        points_left[num_left-1].x-points_left[0].x
    );
    if (PRINT_LASER_MSGS) ROS_INFO("    angle left: %f  (from %d points)",angle_left,num_left);
    angle_left_avg=lerp(angle_left_avg,angle_left,avg_merge);
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
  ros::init(argc, argv, "youbot_whackamole_robot_position_node");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

  ros::Subscriber laser_sub = n.subscribe("base_scan", 1000, laserCallback);

  ros::Subscriber position_sub = n.subscribe("whackamole/cmd_robot_pos", 1000, robotPositionCallback);

  dist_left_avg=DIST_LEFT_DESIRED;
  angle_left_avg=ANGLE_LEFT_DESIRED;
  dist_front_avg=(DIST_FRONT_RANGE_MIN+DIST_FRONT_RANGE_MAX)/2.0f;

  ros::Rate loop_rate(10.0);

  while (ros::ok())
  {
    ROS_INFO("dleft: %fm,  dfront: %fm, aleft: %frad",
      dist_left_avg, dist_front_avg, angle_left_avg);

    //if trying to move to a position,
    if (desired_position>=0 && desired_position<NUM_ROBOT_POSITIONS)
    {

      //check if outside acceptable positions
      bool in_range=true;
      if (dist_left_avg<DIST_LEFT_RANGE_MIN)
      {
        in_range=false;
        ROS_WARN("Too close to wall on left. %f < %f", dist_left_avg,DIST_LEFT_RANGE_MIN);
      }
      if (dist_left_avg>DIST_LEFT_RANGE_MAX)
      {
        in_range=false;
        ROS_WARN("Too far from wall on left. %f > %f", dist_left_avg,DIST_LEFT_RANGE_MAX);
      }
      if (dist_front_avg<DIST_FRONT_RANGE_MIN)
      {
        in_range=false;
        ROS_WARN("Too close to wall on front. %f < %f", dist_front_avg,DIST_FRONT_RANGE_MIN);
      }
      if (dist_front_avg>DIST_FRONT_RANGE_MAX)
      {
        in_range=false;
        ROS_WARN("Too far from wall on front. %f < %f", dist_front_avg,DIST_FRONT_RANGE_MAX);
      }
      if (fabs(angle_left_avg-ANGLE_LEFT_DESIRED) > ANGLE_LEFT_ACCEPTABLE_RANGE)
      {
        in_range=false;
        ROS_WARN("Not parallel enough to wall. %f", angle_left_avg);
      }

      if (!in_range) {
        ROS_WARN("Outside of valid positions, preventing movement");
        geometry_msgs::Twist v;
        v.linear.x=0;
        v.linear.y=0;
        v.angular.z=0;
        cmd_vel_pub.publish(v);
        desired_position=-1;
      }
      else {
        //move to desired position
        //front distance down, linear x positive (negative error)
        //left distance down, linear y positive (negative error)
        //left angle down, angular z negative (positive error)
        geometry_msgs::Twist v;
        bool settled=true;
        //correct left distance
        float error = DIST_LEFT_DESIRED-dist_left_avg;
        if (fabs(error)>DIST_LEFT_SETTLE_THRESHOLD)
        {
          settled=false;
          v.linear.y = aclamp(MIN_Y_VELOCITY, error*-Y_VELOCITY_KP, MAX_Y_VELOCITY);
        }
        else v.linear.y=0;
        //correct front distance
        error = DIST_FRONT_POSITIONS[desired_position]-dist_front_avg;
        if (fabs(error)>DIST_FRONT_SETTLE_THRESHOLD)
        {
          settled=false;
          v.linear.x = aclamp(MIN_X_VELOCITY, error*-X_VELOCITY_KP, MAX_X_VELOCITY);
        }
        else v.linear.x=0;
        //correct angle
        error = ANGLE_LEFT_DESIRED - angle_left_avg;
        if (fabs(error)>ANGLE_LEFT_SETTLE_THRESHOLD)
        {
          settled=false;
          v.angular.z = aclamp(MIN_ANG_VELOCITY, error*ANG_VELOCITY_KP, MAX_ANG_VELOCITY);
        }
        else v.angular.z=0;

        cmd_vel_pub.publish(v);

        if (settled) desired_position=-1;
      }

    }

    loop_rate.sleep();
    ros::spinOnce();
  }

}
