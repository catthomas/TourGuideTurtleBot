#include "math.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "kobuki_msgs/BumperEvent.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
 
enum CurrentAction
{
  CRUISE,
  HALT,
  TURN,
  AVOID_LEFT,
  AVOID_RIGHT,
  ESCAPE,
  TELEOP
};
 
/* Variables for movement */
geometry_msgs::Twist twist;
ros::Publisher halt_publisher;
ros::Publisher escape_publisher;
ros::Publisher avoid_publisher;
ros::Publisher turn_publisher;
ros::Publisher cruise_publisher;
 
/* Helper Variables */
uint8_t bumper;
geometry_msgs::Pose current_pose;
 
// Must be set to current_pose every time we escape, avoid, halt
geometry_msgs::Pose prev_pose;
 
float goal_turn_degree;
CurrentAction action = CRUISE;
 
/* Methods */
void publishHalt();
void publishEscape();
void publishAvoid();
void publishTurn();
void publishCruise();
 
// define the callback functions for topics we'll subscribe to
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &bump_in);
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_in);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_in);
 
/* MAIN: START ALL PROCESSES */
int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebot_navigation");
  ros::NodeHandle n;
  ros::Rate loop_rate(15);
  srand(time(NULL));

/* Subscribers (Sensors) */
  ros::Subscriber bumper_subscriber =
      n.subscribe("mobile_base/events/bumper", 1000, bumperCallback);
  ros::Subscriber odom_subscriber = n.subscribe("odom", 1000, odomCallback);
  ros::Subscriber laser_subscriber = n.subscribe("scan", 1000, laserCallback);
  ros::Subscriber map_subscriber = n.subscribe("map", 1000, mapCallback);
 
  /* Publishers (Commands) */
  halt_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/halt", 1000);
  escape_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/escape", 1000);
  avoid_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/avoid", 1000);
  turn_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/turn", 1000);
  cruise_publisher =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/cruise", 1000);
 
  while (ros::ok())
  {
    if (action == HALT)
    {
      // Robot bumped into something
      publishHalt();
    }
    else if (action == ESCAPE)
    {
      // Symetric obstacle detected
      publishEscape();
    }
    else if (action == AVOID_LEFT || action == AVOID_RIGHT)
    {
      // Asymmetric obstacle detected
      publishAvoid();
    }
    else if (action == TURN)
    {
      // Turn randomly every once 1m
      publishTurn();
    }
    else
    {
      // If nothing else, just cruise!
      publishCruise();
    }
 
    // Sleep for the specified loop rate before looping again
    loop_rate.sleep();
 
    // Spin
    ros::spinOnce();
  }
 
  return 0;
}

/* halt */
void publishHalt() {
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
 
  halt_publisher.publish(twist);
}
 
void publishEscape() {
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0.5;
 
  escape_publisher.publish(twist);
}
 
void publishAvoid() {
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
 
  if (action == AVOID_LEFT)
  {
    twist.angular.z = -0.5;
  }
  else
  {
    twist.angular.z = 0.5;
  }
 
  avoid_publisher.publish(twist);
 
  // Reset back to cruise. If needed, we'll go back to avoid
  action = CRUISE;
}
 
void publishTurn() { 
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = goal_turn_degree > 0 ? 0.5 : -0.5;
 
  turn_publisher.publish(twist);
}
 
void publishCruise() {
  twist.linear.x = 0.35;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
 
  cruise_publisher.publish(twist);
}

//Fetch the sensor state
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &bump_in) {
  ROS_INFO("bumper hit. value = [%d] Halting forever!", bump_in->bumper);
   
  bumper = bump_in->bumper;
  if (bumper != 0)
  {
    action = HALT;
  }
}

// Fetch the odometry information
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_in) {
  current_pose = odom_in->pose.pose;
 
  // Distance
  // Find Euclidian Distance
  double dist = sqrt(pow(current_pose.position.x - prev_pose.position.x, 2.0) +
                     pow(current_pose.position.y - prev_pose.position.y, 2.0) +
                     pow(current_pose.position.z - prev_pose.position.z, 2.0));
 
  // Orientation
  tf::Pose tf_current_pose, tf_prev_pose;
  tf::poseMsgToTF(current_pose, tf_current_pose);
  tf::poseMsgToTF(prev_pose, tf_prev_pose);
 
  // Current rotation - previous rotation, converted from radians
  double turnDegrees =
      (tf::getYaw(tf_current_pose.getRotation()) -
       tf::getYaw(tf_prev_pose.getRotation())) *
      180 / 3.1415928;
   
  if ((action == TURN || action == ESCAPE) && abs(turnDegrees) >= abs(goal_turn_degree))
  {
    action = CRUISE;
    prev_pose = current_pose;
  }
  else if (dist >= 1.0)
  {
    // We've travelled 1 meter. Set up the turn we're about to do
    prev_pose = current_pose;
    action = TURN;
     
    // Pick a random angle between 15 and -15
    goal_turn_degree = (rand() % 31) - 15;
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
  /*
   * Find the distance on the left, right, and in the middle.
   * Truncate the distance array on one side so we're looking at symmetrical angles
   */
  double left_distance, right_distance, middle_distance;
  int left_index, right_index, middle_index;  
 
  // find the smaller angle away from 0 (left or right)
  double min_arc = fmin(fabs(scan_in->angle_min), fabs(scan_in->angle_max));
 
  // find the difference between the absolute value of the min and max angles
  // divide this by the angle increment
  // this gives us the number of indices to ignore at one end of the range
  int index_offset = fabs(fabs(scan_in->angle_min) - fabs(scan_in->angle_max)) / scan_in->angle_increment;
 
  // if the right side is smaller
  if (min_arc == fabs(scan_in->angle_min))
  {
    right_index = 0;
    left_index = scan_in->ranges.size()-1-index_offset;
    middle_index = (scan_in->ranges.size()-index_offset)/2;
  }
  else
  {
  	// if the left side is smaller
    left_index = scan_in->ranges.size()-1;
    right_index = index_offset;
    middle_index = (scan_in->ranges.size()+index_offset)/2;
  }
         
  left_distance = scan_in->ranges[left_index];
  right_distance = scan_in->ranges[right_index];
  middle_distance = scan_in->ranges[middle_index];
 
  /*
   * Handle the obstacle distances, if any
   */
 
  if (left_distance <= 1.0 && right_distance <= 1.0 && middle_distance <= 1.0 &&
    	abs(left_distance - right_distance) <= 0.03)
  {
    // symmetric obstacle detected at every angle
    action = ESCAPE;
    goal_turn_degree = 30;
  }
  else if (middle_distance <= 1.0)
  {
    // symmetric obstacle detected by only the middle
    action = ESCAPE;
    goal_turn_degree = 30;
  }
  else if (left_distance <= 1.0)
  {
    // asymmetric object detected on the left
    action = AVOID_LEFT;
  }
  else if (right_distance <= 1.0)
  {
    // asymmetric object detected on the right
    action = AVOID_RIGHT;
  }
}
 
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_in) {
  //ROS_INFO("Started mapping");
}
