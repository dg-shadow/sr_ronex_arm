/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "sr_ronex_arm_controllers/position_velocity_controller.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( ronex::PositionVelocityController, pr2_controller_interface::Controller)

using namespace std;

namespace ronex {

PositionVelocityController::PositionVelocityController()
: joint_state_(NULL), command_(0), robot_(NULL), last_time_(0), loop_count_(0), velocity_buffer_(50)
{
}

PositionVelocityController::~PositionVelocityController()
{
  sub_command_.shutdown();
}

void PositionVelocityController::starting()
{
  command_ = joint_state_->position_;
  velocity_pid_controller_.reset();
  position_pid_controller_.reset();
  for (velocity_index_ = 0; velocity_index_ < velocity_buffer_.size(); velocity_index_++)
    velocity_buffer_[velocity_index_] = 0.0;
  velocity_sum_ = 0;
  last_position_ = command_;
  joint_state_->commanded_effort_ += effort_demand;
}

bool PositionVelocityController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{

  assert(robot);
  node_ = n;
  robot_ = robot;

  std::string joint_name;

  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

ROS_INFO("%s", joint_name.c_str());

  if (!(joint_state_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!velocity_pid_controller_.init(ros::NodeHandle(node_, "vpid")))
    return false;

  if (!position_pid_controller_.init(ros::NodeHandle(node_, "ppid")))
    return false;

  position_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "position_loop_state", 1));

  velocity_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "velocity_loop_state", 1));


  sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &PositionVelocityController::setCommandCB, this);

  return true;
}


void PositionVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  velocity_pid_controller_.setGains(p,i,d,i_max,i_min);

}

void PositionVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  velocity_pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string PositionVelocityController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint velocity command
void PositionVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void PositionVelocityController::getCommand(double  & cmd)
{
  cmd = command_;
}

double PositionVelocityController::get_velocity_()
{
  velocity_sum_ -= velocity_buffer_[velocity_index_];
  velocity_sum_+= (velocity_buffer_[velocity_index_] =  (joint_state_->position_ - last_position_ ) / dt_.toSec());
  velocity_index_ = (velocity_index_ + 1) % velocity_buffer_.size();
  return velocity_sum_/velocity_buffer_.size();
}

void PositionVelocityController::update()
{
  assert(robot_ != NULL);
  ros::Time time = robot_->getTime();
  dt_ = time - last_time_;

  joint_state_->velocity_ = get_velocity_();

  if      (command_ > joint_state_->joint_->limits->upper) command_ = joint_state_->joint_->limits->upper;
  else if (command_ < joint_state_->joint_->limits->lower) command_ = joint_state_->joint_->limits->lower;


  double position_error = command_ - joint_state_->position_;
  double velocity_demand = position_pid_controller_.computeCommand(position_error, dt_);

  if      (velocity_demand >  joint_state_->joint_->limits->velocity) velocity_demand =  joint_state_->joint_->limits->velocity;
  else if (velocity_demand < -joint_state_->joint_->limits->velocity) velocity_demand = -joint_state_->joint_->limits->velocity;

  double velocity_error = velocity_demand - joint_state_->velocity_;
  double effort_demand = velocity_pid_controller_.computeCommand(velocity_error , dt_);

  joint_state_->commanded_effort_ += effort_demand;

  if(loop_count_ % 10 == 0)
  {
    if(velocity_state_publisher_ && velocity_state_publisher_->trylock())
    {
      velocity_state_publisher_->msg_.header.stamp = time;
      velocity_state_publisher_->msg_.set_point = velocity_demand;
      velocity_state_publisher_->msg_.process_value = joint_state_->velocity_;
      velocity_state_publisher_->msg_.error = velocity_error;
      velocity_state_publisher_->msg_.time_step = dt_.toSec();
      velocity_state_publisher_->msg_.command = effort_demand;

      double dummy;
      velocity_pid_controller_.getGains(
               velocity_state_publisher_->msg_.p,
               velocity_state_publisher_->msg_.i,
               velocity_state_publisher_->msg_.d,
               velocity_state_publisher_->msg_.i_clamp,
               dummy);
      velocity_state_publisher_->unlockAndPublish();
    }
    if(position_state_publisher_ && position_state_publisher_->trylock())
    {
      position_state_publisher_->msg_.header.stamp = time;
      position_state_publisher_->msg_.set_point = command_;
      position_state_publisher_->msg_.process_value = joint_state_->position_;
      position_state_publisher_->msg_.error = position_error;
      position_state_publisher_->msg_.time_step = dt_.toSec();
      position_state_publisher_->msg_.command = velocity_demand;

      double dummy;
      position_pid_controller_.getGains(position_state_publisher_->msg_.p,
               position_state_publisher_->msg_.i,
               position_state_publisher_->msg_.d,
               position_state_publisher_->msg_.i_clamp,
               dummy);
      position_state_publisher_->unlockAndPublish();
    }

  }
  loop_count_++;

  last_time_ = time;
  last_position_ = joint_state_->position_;
}

void PositionVelocityController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

} // namespace

