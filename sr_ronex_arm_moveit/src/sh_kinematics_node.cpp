/**
 * @file   sr_ronex_arm_moveit_node.cpp
 * @author Yi Li <yi@shadowrobot.com>
 *
 * Copyright (c) 2014 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 *
 * @brief Move the "the_arm" group.
 *
 *
 */

#include <moveit/move_group_interface/move_group.h>

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  std::string node_name("sr_ronex_arm_moveit_node");
  ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Sleep for a few second and wait for RViz.
  ros::Duration(12.0).sleep();

  // this connecs to a running instance of the move_group node
  move_group_interface::MoveGroup group("the_arm");
  // specify that our target will be a random one
  group.setRandomTarget();
  // plan the motion and then move the group to the sampled target
  group.move();
  ros::waitForShutdown();
}

//-------------------------------------------------------------------------------
