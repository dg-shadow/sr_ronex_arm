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
 * @brief  A passthrough for the General IO RoNeX module: simply sets 
 *         the different pins to the value you want directly.
 **/

#ifndef _POSITION_SEVRO_CONTROLLER_H_
#define _POSITION_SERVO_CONTROLLER_H_

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sr_ronex_msgs/PWM.h>

namespace ronex
{
  class PositionServoController
    : public pr2_controller_interface::Controller
  {
  public:
    PositionServoController();
    virtual ~PositionServoController();

    virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);
    bool init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name);

    void setCommand(double cmd);
    void getCommand(double & cmd);

    virtual void starting();
    virtual void update();
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);


  private:
    ros::NodeHandle node_;

    int loop_count_;
    double upper_limit_, lower_limit_, command_;
    pr2_mechanism_model::JointState *joint_state_; 
    pr2_mechanism_model::RobotState *robot_;
    ros::Time last_time_;
    bool initialised_;
ros::Subscriber sub_command_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_ */
