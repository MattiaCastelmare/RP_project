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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


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

  // make_shared creates a shared pointer that manages a new object ref:https://en.cppreference.com/w/cpp/memory/shared_ptr 
  map_ptr = std::make_shared<Map>(); 
  
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

  // Map subscriber
  ros::Subscriber map_subscriber = nh.subscribe("/map", 10, callback_map); 

  // Initial pose subscriber
  ros::Subscriber initial_pose_subscriber = nh.subscribe("/initialpose", 10, callback_initialpose); 

  // Base scan subscriber
  ros::Subscriber base_scan_subscriber = nh.subscribe("/base_scan", 10, callback_scan); 

  // Advertise the topic /odom_out [nav_msgs::Odometry]
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
  // Set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  // TODO

  // If map_ptr exists and it is not initialized
  if (map_ptr && !map_ptr->initialized()) 
    { 
      // Loads the map (occupancyGrid) 
      map_ptr->loadOccupancyGrid(msg_); 

      // ROS message
      ROS_INFO("Map loaded!"); 

      // Setting the localizer map
      localizer.setMap(map_ptr);  
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

  // I have to access the pose from the msg_
  geometry_msgs::Pose pose_obj = msg_ -> pose.pose; 

  // Create Eigen Isometry called inital_pose
  Eigen::Isometry2f initial_pose; 

  // Convert geometry_msgs into Eigen isometry
  pose2isometry(pose_obj, initial_pose);

  // Informing the localizer of the initial pose  
  localizer.setInitialPose(initial_pose); 

  // ROS message for the initial pose
  ROS_INFO("Initial pose: x=%f, y=%f, theta=%f",
           initial_pose.translation().x(),
           initial_pose.translation().y(),
           Eigen::Rotation2Df(initial_pose.linear()).angle());

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

  // parameters are set in the localizer2d.h file
  float range_min = msg_ -> range_min; 
  float range_max = msg_ -> range_max; 
  float angle_min = msg_ -> angle_min; 
  float angle_max = msg_ -> angle_max; 
  float angle_increment = msg_ -> angle_increment; 

  // Setting the laser parameters
  localizer.setLaserParams(range_min, range_max, angle_min, angle_max, angle_increment);

  // Process the incoming scan
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

  // Call the localizer X() and return the current pose of the laser in the world
  Eigen::Isometry2f laser_in_world = localizer.X(); 

  // Create a message for the pose of the lase in the map
  geometry_msgs::TransformStamped laser_message; 

  // Convert the message of the isometry of the laser into the right message
  isometry2transformStamped(laser_in_world, laser_message, FRAME_WORLD, FRAME_LASER, msg_->header.stamp); 

  // Send a transform message
  br.sendTransform(laser_message); 

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  // TODO

  // Create a nav_msgs object type
  nav_msgs::Odometry odom_laser_in_world; 

  // Convert the Stamped into an odometry
  transformStamped2odometry(laser_message, odom_laser_in_world); 

  // Publish the odometry by the Publisher
  pub_odom.publish(odom_laser_in_world); 

  // Log the odom_laser_in_world information
  ROS_INFO("Odometry (laser in world): x=%f, y=%f, theta=%f",
           odom_laser_in_world.pose.pose.position.x,
           odom_laser_in_world.pose.pose.position.y,
           tf2::getYaw(odom_laser_in_world.pose.pose.orientation));

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);

  // Log the out_scan information
  ROS_INFO("Outgoing LaserScan frame_id: %s", out_scan.header.frame_id.c_str());
}