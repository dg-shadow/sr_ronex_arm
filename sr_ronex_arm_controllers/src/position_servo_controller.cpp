/*
 * Copyright (c) 2013, Shadow Robot Company, All rights reserved.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

/**
 * @file   general_io_passthrough_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A passthrough for the General IO RoNeX module: simply sets the
 *         different pins to the value you want directly.
 **/

#include "sr_ronex_arm_controllers/position_servo_controller.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( ronex::PositionServoController, pr2_controller_interface::Controller)

namespace ronex
{
  PositionServoController::PositionServoController(): joint_state_(NULL), command_(0),
  loop_count_(0), initialised_(false), robot_(NULL), last_time_(0)
  {}

  PositionServoController::~PositionServoController()
  {
  }

  bool PositionServoController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name)
  {
    assert(robot);

    robot_ = robot;

    last_time_ = robot_->getTime();
    joint_state_ = robot_->getJointState(joint_name);

    if (!joint_state_)
      {
        ROS_ERROR("JointPositionController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }

    if (!joint_state_->calibrated_)
      {
        ROS_ERROR("Joint %s not calibrated for JointPositionController", joint_name.c_str());
        return false;
      }

    state_publisher_.reset(
			   new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
			   (node_, "state", 1));

    upper_limit_ = joint_state_->joint_->limits->upper;
    lower_limit_ = joint_state_->joint_->limits->lower;

    command_ = 0;

    return true;
  }

  bool PositionServoController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;
  
    std::string joint_name;

    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &PositionServoController::setCommandCB, this);


    return init(robot, joint_name);

  }

  void PositionServoController::setCommand(double cmd)
  {
    command_ = cmd;
  }

  void PositionServoController::getCommand(double & cmd)
  {
    cmd = command_;
  }

  void PositionServoController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    command_ = msg->data;
  }

  void PositionServoController::starting()
  {}

  /*
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void PositionServoController::update()
  {
    ros::Time now = robot_->getTime();
    dt_ = now - last_time_;



    
    double max_step = joint_state_->joint_->limits->velocity * dt_.toSec();

    double input = command_;
    if      (input > upper_limit_) input = upper_limit_;
    else if (input < lower_limit_) input = lower_limit_;

    if      (input > joint_state_->position_ + max_step) input = joint_state_->position_ + max_step;
    else if (input < joint_state_->position_ - max_step) input = joint_state_->position_ - max_step;

    if(loop_count_ % 10 == 0)
    {
      loop_count_ = 0;
      if(state_publisher_ && state_publisher_->trylock())
	{
	  state_publisher_->msg_.header.stamp = now;
	  state_publisher_->msg_.set_point = command_;
	  state_publisher_->msg_.process_value = joint_state_->position_;
	  state_publisher_->msg_.error = 0;
	  state_publisher_->msg_.time_step = dt_.toSec();
	  state_publisher_->msg_.command = input;
	 
	  state_publisher_->unlockAndPublish();
	}
      
    }
    loop_count_++;    


    double input_range = (upper_limit_ - lower_limit_);

    double output = ( (input - lower_limit_) / input_range * 200) - 100; // scales from range lower_limit_ -> upper_limit_ to output_lower_-> output_higher_ (-100 -> +100)

    joint_state_->commanded_effort_ = output;

    last_time_ = robot_->getTime();
    joint_state_->position_ = input;

 
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
