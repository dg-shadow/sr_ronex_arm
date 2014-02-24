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
* @file command_to_pwm.hpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
* @brief Contains the data mapping one joint command to a pwm module.
**/

#include "sr_ronex_arm_transmissions/mapping/general_io/command_to_servo.hpp"
#include <pr2_mechanism_model/robot.h>
#include <boost/lexical_cast.hpp>
#include <math.h>

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
       CommandToServo::CommandToServo(TiXmlElement* mapping_el, pr2_mechanism_model::Robot* robot)
        : RonexMapping(), pin_out_of_bound_(true)
      {

        const char *ronex_name = mapping_el ? mapping_el->Attribute("ronex") : NULL;
        if (!ronex_name)
        {
          ROS_ERROR("RonexArmTransmission transmission did not specify the ronex name");
          return;
        }

        general_io_ = static_cast<ronex::GeneralIO*>( robot->hw_->getCustomHW(ronex_name) );
        if(!general_io_)
        {
          ROS_ERROR_STREAM("The RoNeX: " << ronex_name << " was not found on the system.");
          return;
        }

        //read PWM module index from urdf
        const char *pwm_module = mapping_el ? mapping_el->Attribute("pwm_module") : NULL;
        if (!pwm_module)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the pwm module.");
          return;
        }
        //convert to size_t
        try
        {
          pwm_module_ = boost::lexical_cast<size_t>( pwm_module );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pwm_module to a size_t.");
          return;
        }

        //read PWM pin index from urdf
        const char *pin = mapping_el ? mapping_el->Attribute("pwm_pin") : NULL;
        if (!pin)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the pwm pin.");
          return;
        }
        //convert to size_t
        try
        {
          pwm_pin_index_ = boost::lexical_cast<size_t>( pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pwm_pin to a size_t.");
          return;
        }
        
        const char *max_on_time = mapping_el ? mapping_el->Attribute("max_on_time") : NULL;
        if (!max_on_time)
        {
          ROS_ERROR("RonexArmTransmission transmission did not specify the max ontime");
          return;
        }

        try
        {
          max_on_time_ = boost::lexical_cast<long int>( max_on_time );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse ontime to an int.");
          return;
        }

        const char *min_on_time = mapping_el ? mapping_el->Attribute("min_on_time") : NULL;
        if (!min_on_time)
        {
          ROS_ERROR("RonexArmTransmission transmission did not specify the min ontime");
          return;
        }
	    try
        {
          min_on_time_ = boost::lexical_cast<long int>( min_on_time );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse ontime to an int.");
          return;
        }


      }

      CommandToServo::~CommandToServo()
      {
      }

      bool CommandToServo::check_pins_in_bound_()
      {
        //we ignore the first iteration as the array is not yet initialised.
        if( first_iteration_ )
        {
          pin_out_of_bound_ = true;
          first_iteration_ = false;
          return false;
        }

        //we have to check here for the size otherwise the general io hasn't been updated.
        if( pin_out_of_bound_ )
        {
          if( pwm_module_ >= general_io_->command_.pwm_.size() )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified PWM module index is out of bound: " << pwm_pin_index_ << " / max = " << general_io_->command_.pwm_.size() << ", not propagating the command to the RoNeX.");
            pin_out_of_bound_ = true;
            return false;
          }
          if( pwm_pin_index_ > 1 )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified PWM pin is out of bound: " << pwm_pin_index_ << " / max = 1, not propagating the command to the RoNeX.");
            pin_out_of_bound_ = true;
            return false;
          }

        }

        pin_out_of_bound_ = false;
        return true;
      }

      void CommandToServo::propagateToRonex(std::vector<pr2_mechanism_model::JointState*>& js)
      {
        assert(js.size() == 1);
        if( check_pins_in_bound_() )
        {
	      unsigned short int on_time = compute_on_time_(js[0]->commanded_effort_);
          general_io_->command_.pwm_[pwm_module_].period = 64000;

          if( pwm_pin_index_ == 0 )
            general_io_->command_.pwm_[pwm_module_].on_time_0 = on_time;
          else
            general_io_->command_.pwm_[pwm_module_].on_time_1 = on_time;
  
       }

      }
      unsigned short int CommandToServo::compute_on_time_ (double & effort)
      {
        if      (effort >  100.0) effort =  100.0;
        else if (effort < -100.0) effort = -100.0;

        return (unsigned short int) ( ( (effort + 100.0) / 200.0 ) * double(max_on_time_ - min_on_time_) + double(min_on_time_) );
      }
    }
  }
}
/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/
