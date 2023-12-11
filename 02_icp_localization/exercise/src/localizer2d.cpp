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
  //std::vector<Eigen::Vector2f> _obst_vect; // create the vector for the obstacle

  if (_map && _map -> initialized())
    { 
      int rows = _map -> rows(); // number of rows of the map
      int cols = _map -> cols(); // number of cols of the map
      _obst_vect.clear();
      // std::vector<int8_t> vector_map = _map -> grid(); // convert the map from a 2D array to an std::vector<int8_t> in which each element is a CellType (-1 0 100) 

      for (size_t x = 0; x < rows; ++x) // iterate all over the rows
      {
      for (size_t y = 0; y < cols; ++y) // iterate all over the columns
            if ((*_map)(x,y) == CellType::Occupied) // check if the cell is occupied
             {
              Eigen::Vector2f world_point = _map -> grid2world(cv::Point2i (x,y)); // convert from map point to world coordinates
              // std::cout << "world coordinates are " << world_point << std::endl;
              _obst_vect.push_back(world_point);
             }
            else
              continue;
       }
  }

  // Create KD-Tree
  // TODO 
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end()); // _obst_tree_ptr is a shared pointer
  std::cout << "KD Tree created!" << std::endl;
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO
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
  ContainerType synthetic_scan;
  getPrediction(synthetic_scan); // this function will populate the synthetic_scan vector with the computed prediction
  

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO
  int min_points_in_leaf = 100;
  ICP icp(synthetic_scan, scan_, min_points_in_leaf); // setting ICP with prediction and scan

  icp.X() = _laser_in_world; // I set the initial guess for the ICP as the current estimate of the laser in world

  int num_iterations = 20; // number of iterations of the ICP
  icp.run(num_iterations); // running the ICP algorithm for 100 iterations

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO
  _laser_in_world = icp.X(); // I store the solution of the ICP algorithm as the current estimate of the laser in world
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
  float ball_radius = 20;
  // Query the KD-Tree to find neighbors within the search radius around the current laser_in_world estimate
  std::vector<PointType*> neighbors;
  _obst_tree_ptr->fullSearch(neighbors, _laser_in_world.translation(), ball_radius);

  for (const auto& neighbor : neighbors) {
    // check if the neighbor is within the range min and range max and if it is within the angle min and angle max
    float x_laser = _laser_in_world.translation().x();
    float y_laser = _laser_in_world.translation().y();

    float x_neighbor = neighbor -> x();
    float y_neighbor = neighbor -> y();

    float diff_x = x_laser - x_neighbor;
    float diff_y = y_laser - y_neighbor; 

    float distance = sqrt(std::pow(diff_x,2) + std::pow(diff_y,2)); // compute the distance between the prediction and the laser
    float angle = std::atan2(diff_y, diff_x); // compute the angle between the prediction and the laser
    prediction_.push_back(*neighbor);
    if ((_range_min <= distance && distance <= _range_max) &&
        (_angle_min <= angle && angle <= _angle_max)) {

          prediction_.push_back(*neighbor);
       }   
  }
}

