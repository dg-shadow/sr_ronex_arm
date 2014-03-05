/**
 * @file   sh_kinematics.hpp
 * @author Yi Li <yi@shadowrobot.com>
 *
 * Copyright (c) 2014 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 *
 * @brief Sample states for a given group using MoveIt!
 *
 *
 */

#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace shadowrobot {

class ShKinematics {
public:
  ShKinematics();
  ~ShKinematics();

  void sample_random_positions(const std::string group_name,
                               const unsigned int no_of_samples,
                               bool visualize = false,
                               double visu_loop_rate = 1.0);

  void sample_random_positions_near_by(const std::string group_name,
                                       const unsigned int no_of_samples,
                                       const double distance,
                                       bool visualize = false,
                                       double visu_loop_rate = 1.0);

private:
  ros::NodeHandle nh_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
  boost::shared_ptr<planning_scene::PlanningScene> planning_scene_;
  ros::Publisher robot_state_publisher_;
};

} // namespace shadowrobot

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
