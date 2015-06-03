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
   the joint impedance control strategy.
   It is based on the default HW-I for any simulated robot in Gazebo from 
   gazebo_ros_control package.
*/

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ros_control
#include <lwr_hw/lwr_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace lwr_hw
{

class LWRHWSimPlugin : public gazebo::ModelPlugin
{
public:

  virtual ~LWRHWSimPlugin();

  // Overloaded Gazebo entry point
  virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Called by the world update start event
  void Update();

  // Called on world reset
  virtual void Reset();

  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const;

  // Get Transmissions from the URDF
  bool parseTransmissionsFromURDF(const std::string& urdf_string);

protected:

  // Node Handles
  ros::NodeHandle model_nh_; // namespaces to robot name

  // Pointer to the model
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;

  // deferred load in case ros is blocking
  boost::thread deferred_load_thread_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<lwr_hw::LWRHWsim> > robot_hw_sim_loader_;
  void load_robot_hw_sim_srv();

  // Strings
  std::string robot_namespace_;
  std::string robot_description_;

  // Transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // Robot simulator interface
  std::string robot_hw_sim_type_str_;
  boost::shared_ptr<lwr_hw::LWRHWsim> robot_hw_sim_;

  // Controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

};


}
