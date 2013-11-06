#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define LASER_ANGLE_THRESH_LEFT -0.8f
#define LASER_ANGLE_THRESH_FRONT 0.05f

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
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

  ROS_INFO("    dist left: %f",dist_left);
  ROS_INFO("    dist front: %f",dist_front);

  //approximate line angles
  if (num_left>1)
  {
    float angle_left;
    angle_left=atan2(
        points_left[num_left-1].y-points_left[0].y,
        points_left[num_left-1].x-points_left[0].x
    );
    ROS_INFO("    angle left: %f  (from %d points)",angle_left,num_left);
  }
  else ROS_WARN("   No points found to the left");

  if (num_front>1)
  {
    float angle_front;
    angle_front=atan2(
        points_front[num_front-1].y-points_front[0].y,
        points_front[num_front-1].x-points_front[0].x
    );
    ROS_INFO("    angle front: %f  (from %d points)",angle_front,num_front);
  }
  else ROS_WARN("   No points found to the front");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_whackamole_robot_position_node");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

  ros::Subscriber laser_sub = n.subscribe("base_scan", 1000, laserCallback);

  ros::spin();
}
