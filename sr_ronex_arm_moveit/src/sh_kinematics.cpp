/**
 * @file   sh_kinematics.cpp
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

#include "sr_ronex_arm_moveit/sh_kinematics.hpp"

namespace shadowrobot {

//-------------------------------------------------------------------------------

ShKinematics::ShKinematics()
{
  // Robot Model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  // Kinematic Model
  kinematic_model_ = robot_model_loader.getModel();
  ROS_DEBUG_STREAM("Model frame: " << kinematic_model_->getModelFrame());

  // Kinematic State
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();

  // This is not the recommended way to instantiate a PlanningScene/
  // The :planning_scene_monitor:'PlanningSceneMonitor' is the recommended
  // method to create and maintain the current planning scene
  // using data from the robot's joints and the sensors on the robot.
  planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));

  robot_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("sr_ronex_arm_state", 1);
}

//-------------------------------------------------------------------------------

ShKinematics::~ShKinematics()
{
}

//-------------------------------------------------------------------------------

void ShKinematics::run(void)
{
  this->inverse_kinematics();

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce(); // Handle ROS events
    r.sleep();
  }
}

//-------------------------------------------------------------------------------

void ShKinematics::inverse_kinematics(void)
{
  std::string group_name("the_arm");
  const robot_state::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);

  /* Compute FK for a set of random joint values*/
  kinematic_state_->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform("link_5");

  /* Get the joint values*/
  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i=0; i < joint_values.size(); ++i)
  {
    ROS_INFO("Joint (FK): %f", joint_values[i]);
  }

  ROS_ERROR_STREAM("+++++++++++++++++++++++++++++++++++");
  /* Inverse Kinematics */
  bool found_ik = kinematic_state_->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
  ROS_ERROR_STREAM("+++++++++++++++++++++++++++++++++++");

  if (found_ik)
  {
    kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_values.size(); ++i)
    {
      ROS_INFO("Joint (IK): %f", joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  ROS_ERROR_STREAM("+++++++++++++++++++++++++++++++++++");
}

//-------------------------------------------------------------------------------

void ShKinematics::sample_random_positions(const std::string group_name,
                                           const unsigned int no_of_samples,
                                           bool visualize,
                                           double visu_loop_rate)
{
  ROS_DEBUG_STREAM("*** Sample Random Positions ***");
  ROS_DEBUG_STREAM("*******************************");

  const robot_state::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);

  // For self-collision checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  ros::Rate loop_rate(visu_loop_rate);
  for (unsigned int i=0; i<no_of_samples && ros::ok(); i++)
  {
    kinematic_state_->setToRandomPositions(joint_model_group);

    // Are there any collisions between the group and other parts of the robot?
    collision_request.group_name = group_name;
    collision_result.clear();
    planning_scene_->checkSelfCollision(collision_request, collision_result);
    ROS_DEBUG_STREAM("Current state is "
                     << (collision_result.collision ? "in" : "not in")
                     << " self collision");

    if (visualize)
    {
      ROS_DEBUG_STREAM("Visualizing the kinematic state (pose) of the robot..");
      moveit_msgs::DisplayRobotState msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state_, msg.state);
      robot_state_publisher_.publish( msg );
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

//-------------------------------------------------------------------------------

void ShKinematics::sample_random_positions_near_by(const std::string group_name,
                                                   const unsigned int no_of_samples,
                                                   const double distance,
                                                   bool visualize,
                                                   double visu_loop_rate)
{
  ROS_DEBUG_STREAM("*** Sample Random Positions Near By ***");
  ROS_DEBUG_STREAM("***************************************");

  const robot_state::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
  kinematic_state_->setToDefaultValues(joint_model_group, "index_thumb_open_pose");
  robot_state::RobotState near_state(*kinematic_state_);

  // For self-collision checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  ros::Rate loop_rate(visu_loop_rate);
  for (unsigned int i=0; i<no_of_samples && ros::ok(); i++)
  {
    kinematic_state_->setToRandomPositionsNearBy(joint_model_group, near_state, distance);

    // Are there any collisions between the group and other parts of the robot?
    collision_request.group_name = group_name;
    collision_result.clear();
    planning_scene_->checkSelfCollision(collision_request, collision_result);
    ROS_DEBUG_STREAM("Current state is "
                     << (collision_result.collision ? "in" : "not in")
                     << " self collision");

    if (visualize)
    {
      ROS_DEBUG_STREAM("Visualizing the kinematic state (pose) of the robot..");
      moveit_msgs::DisplayRobotState msg;
      robot_state::robotStateToRobotStateMsg(*kinematic_state_, msg.state);
      robot_state_publisher_.publish( msg );
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}

//-------------------------------------------------------------------------------

} // namespace shadowrobot
