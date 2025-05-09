/**
 * *********************************************************
 *
 * @file: mpc_controller.h
 * @brief: Contains the model predicted control (MPC) local controller class
 * @author: Yang Haodong
 * @date: 2024-01-31
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_CONTROLLER_MPC_CONTROLLER_H_
#define RMP_CONTROLLER_MPC_CONTROLLER_H_

#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>

#include <Eigen/Dense>

#include "controller/controller.h"

namespace rmp
{
namespace controller
{
/**
 * @brief A class implementing a local planner using the MPC
 */
class MPCController : public nav_core::BaseLocalPlanner, Controller
{
public:
  /**
   * @brief Construct a new MPC planner object
   */
  MPCController();

  /**
   * @brief Construct a new MPC planner object
   */
  MPCController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the MPC planner object
   */
  ~MPCController();

  /**
   * @brief Initialization of the local planner
   * @param name        the name to give this instance of the trajectory planner
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Check if the goal pose has been achieved
   * @return true if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  /**
   * @brief Execute MPC control process
   * @param s     current state
   * @param s_d   desired state
   * @param u_r   refered control
   * @param du_p  previous control error
   * @return u  control vector
   */
  Eigen::Vector2d _mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r, Eigen::Vector2d du_p);

private:
  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  double d_t_;            // control time interval
  Eigen::Matrix3d Q_;     // state error matrix
  Eigen::Matrix2d R_;     // control error matrix
  int p_;                 // predicting time domain
  int m_;                 // control time domain
  Eigen::Vector2d du_p_;  // previous control error

  ros::Publisher target_pt_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};
}  // namespace controller
}  // namespace rmp
#endif
