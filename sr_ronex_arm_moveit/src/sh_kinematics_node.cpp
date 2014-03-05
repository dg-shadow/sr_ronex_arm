/**
 * @file   sh_kinematics_node.cpp
 * @author Yi Li <yi@shadowrobot.com>
 *
 * Copyright (c) 2014 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 *
 * @brief A node that maps human hand poses to robotic hand poses.
 *
 *
 */

#include "sr_ronex_arm_moveit/sh_kinematics.hpp"

using namespace shadowrobot;

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  std::string node_name("sr_ronex_arm_moveit_node");
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // This sleep is ONLY to allow Rviz to come up
  sleep(10.0);

  // Call the constructor!
  ShKinematics sh_kinematics;

  // Select the group...
  const std::string group_name("index_finger_thumb");

  // Get params...
  int no_of_samples;
  double conf_distance;
  bool visualize;
  double visu_loop_rate;
  ros::param::get("/" + node_name + "/no_of_samples", no_of_samples);
  ros::param::get("/" + node_name + "/conf_distance", conf_distance);
  ros::param::get("/" + node_name + "/visualize", visualize);
  ros::param::get("/" + node_name + "/visu_loop_rate", visu_loop_rate);

  sh_kinematics.sample_random_positions(group_name,
                                        no_of_samples,
                                        visualize,
                                        visu_loop_rate);

  sh_kinematics.sample_random_positions_near_by(group_name,
                                                no_of_samples,
                                                conf_distance,
                                                visualize,
                                                visu_loop_rate);

  // Shutdown ROS
  ros::shutdown();
  return 0;
}

//-------------------------------------------------------------------------------
