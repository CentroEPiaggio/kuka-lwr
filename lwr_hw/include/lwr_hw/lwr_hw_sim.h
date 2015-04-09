/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>
*/

/* Author: Carlos Rosales <cjrosales@gmail.com> 
   Desc:   Implements a default KUKA LWR 4+ simulation interface that emulates 
   the joint impedance control strategy using the gazebo_ros_control framework.
   It is based on the default HW-I for any simulated robot in Gazebo from 
   gazebo_ros_control package.
*/

#ifndef LWR_HW____LWRs_HW_SIM_H
#define LWR_HW____LWRs_HW_SIM_H

#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

// RRBOT hardware base class
#include "lwr_hw/lwr_hw.h"

namespace lwr_hw {

  // Struct for passing loaded joint data
  struct JointData 
  {
    std::string name_;
    std::string hardware_interface_;

    JointData(const std::string& name, const std::string& hardware_interface) :
      name_(name),
      hardware_interface_(hardware_interface)
    {}
  };

  // Gazebo plugin version of RRBOTHardware
  class LWRHWsim : public lwr_hw::LWRHW
  {
  public:

    virtual ~LWRHWsim() { }

    virtual bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh, 
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions) = 0;

    virtual void readSim(ros::Time time, ros::Duration period) = 0;

    virtual void writeSim(ros::Time time, ros::Duration period) = 0;

  };

}

#endif