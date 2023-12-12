#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"
#include <iostream>
#include <cmath> 

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  // TODO
  
  // Check if _map exists and if it is initialized
  if (_map && _map -> initialized())
    { 
      // Number of rows of the map
      int rows = _map -> rows(); 

      // Number of cols of the map
      int cols = _map -> cols();

      // Clean _obst_vect before populating it 
      _obst_vect.clear();
      
      // Iterate all over the rows
      for (size_t x = 0; x < rows; ++x) 
      {
        // Iterate all over the columns
        for (size_t y = 0; y < cols; ++y) 

            // Check if the cell is occupied
            if ((*_map)(x,y) == CellType::Occupied) 
             {
              // Convert from map point to world coordinates
              Eigen::Vector2f world_point = _map -> grid2world(cv::Point2i (x,y)); 

              _obst_vect.push_back(world_point);
             }
            else
              continue;
       }
  }

  // Create KD-Tree
  // TODO 

  // Instatiate the KD-Tree with std::make_shared because _obst_tree_ptr is a shared pointer
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end()); 
  std::cout << "KD Tree created!" << std::endl;
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO

  // Setting the initial pose as the current pose of the sensor 
  _laser_in_world = initial_pose_;
  
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO

  // Create a container with teh synthetic scan
  ContainerType synthetic_scan;

  // This function will populate the synthetic_scan vector with the computed prediction
  getPrediction(synthetic_scan); 
  

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO

  // Setting the minimum points in a leaf for the KD in the ICP
  int min_points_in_leaf = 10;

  // Setting ICP with prediction and scan
  ICP icp(synthetic_scan, scan_, min_points_in_leaf); 

  // Set the initial guess for the ICP as the current estimate of the laser in world
  icp.X() = _laser_in_world; 

  // Number of iterations of the ICP
  int num_iterations = 20;

  // Running the ICP algorithm  
  icp.run(num_iterations); 

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO

  // Store the solution of the ICP algorithm as the current estimate of the laser in world
  _laser_in_world = icp.X(); 
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO

  // Setting the radius of the searching area for the KD-Tree
  float ball_radius = 20;

  // Creatin neighbors vector to populate with the searching in the KD-Tree
  std::vector<PointType*> neighbors;

  // Query the KD-Tree to find neighbors within the search radius around the current laser_in_world estimate (translation only)
  _obst_tree_ptr->fullSearch(neighbors, _laser_in_world.translation(), ball_radius);

  // Calculate x and y coordinates of the laser 
  float x_laser = _laser_in_world.translation().x();
  float y_laser = _laser_in_world.translation().y();

  // Iterate all over the neighbors in order to put them in the prediction container
  for (const auto& neighbor : neighbors) {

    // Calculate x and y coordinates of the neighbor
    float x_neighbor = neighbor -> x();
    float y_neighbor = neighbor -> y();

    // Calculate differences between laser and neighbor in both x and y
    float diff_x = x_laser - x_neighbor;
    float diff_y = y_laser - y_neighbor; 

    // Calculate the Euclidean distance between neighbor and laser
    float distance = sqrt(std::pow(diff_x,2) + std::pow(diff_y,2)); 

    // Calculate the angle between the neighbor and laser
    float angle = std::atan2(diff_y, diff_x); 

    // If both distance and angle are inside the laser range put them in the prediction container
    if ((_range_min <= distance && distance <= _range_max) &&
        (_angle_min <= angle && angle <= _angle_max)) {
          
          // Put in the container the neighbor
          prediction_.push_back(*neighbor);
       }   
  }
}

