#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;

Localizer2D localizer;

int main(int argc, char** argv) {
  // Initialize ROS system
  // TODO
  ros::init(argc, argv, "localizer_node");

  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");

  // Create shared pointer for the Map object
  // TODO
  map_ptr = std::make_shared<Map>(); // make_shared creates a shared pointer that manages a new object ref:https://en.cppreference.com/w/cpp/memory/shared_ptr 
  
  //
  /**
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   *
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  // TODO
  ros::Subscriber map_subscriber = nh.subscribe("map", 10, callback_map); // map subscriber
  ros::Subscriber initial_pose_subscriber = nh.subscribe("initial_pose", 10, callback_initialpose); // initial pose subscriber
  ros::Subscriber base_scan_subscriber = nh.subscribe("base_scan", 10, callback_scan); // base scan subscriber

  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  // TODO
  if (!map_ptr->initialized())
    {   
      map_ptr->loadOccupancyGrid(msg_); // function inside the map.h 
      ROS_INFO("Maps loaded!"); // ROS messages to indicate that the maps is loaded
      localizer.setMap(map_ptr); // setting the localizer map 
    }
}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */
  // TODO
  geometry_msgs::Pose pose_obj = msg_ -> pose.pose; // I have to access the pose from the msg_

  Eigen::Isometry2f initial_pose; // create Eigen Isometry called inital_pose
  pose2isometry(pose_obj, initial_pose);  // convert geometry_msgs into Eigen isometry
  localizer.setInitialPose(initial_pose); // informing the localizer
  
}

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // TODO
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> scan_points;
  scan2eigen(msg_, scan_points);
  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  // TODO
  // the parameters are set in the localizer2d.h file
  float range_min = msg_ -> range_min; 
  float range_max = msg_ -> range_max; 
  float angle_min = msg_ -> angle_min; 
  float angle_max = msg_ -> angle_max; 
  float angle_increment = msg_ -> angle_increment; 

  localizer.setLaserParams(range_min, range_max, angle_min, angle_max, angle_increment);

  localizer.process(scan_points); 

  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
  static tf2_ros::TransformBroadcaster br;
  // TODO
  Eigen::Isometry2f laser_in_world; // create an object to store the laser pose in the world
  laser_in_world = localizer.X(); // call the localizer X() and return the current pose of the laser in the world

  geometry_msgs::TransformStamped laser_message; // create a message for the pose of the lase in the map

  isometry2transformStamped(laser_in_world, laser_message, FRAME_WORLD, FRAME_LASER, msg_->header.stamp); // convert the message of the isometry of the laser into the right message
  br.sendTransform(laser_message); // send a transform message

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  // TODO
  nav_msgs::Odometry odom_laser_in_world; // create a nav_msgs object type
  transformStamped2odometry(laser_message, odom_laser_in_world); // convert the Stamped into an odometry
  pub_odom.publish(odom_laser_in_world); // publish the odometry by the Publisher

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}