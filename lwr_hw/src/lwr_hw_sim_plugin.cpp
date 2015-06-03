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

// Boost
#include <boost/bind.hpp>

// URDF
#include <urdf/model.h>

#include "lwr_hw/lwr_hw_sim_plugin.h"

namespace lwr_hw
{

LWRHWSimPlugin::~LWRHWSimPlugin()
{
  // Disconnect from gazebo events
  gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

// Overloaded Gazebo entry point
void LWRHWSimPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  ROS_INFO_STREAM_NAMED("lwr_hw","Loading lwr_hw plugin");


  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;

  // Error message if the model couldn't be found
  if (!parent_model_)
  {
    ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("lwr_hw","A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get namespace for nodehandle
  if(sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
  }
  else
  {
    robot_namespace_ = parent_model_->GetName(); // default
  }

  // Get robot_description ROS param name
  if (sdf_->HasElement("robotParam"))
  {
    robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
  }
  else
  {
    robot_description_ = "robot_description"; // default
  }

  // Get the robot simulation interface type
  if(sdf_->HasElement("robotSimType"))
  {
    robot_hw_sim_type_str_ = sdf_->Get<std::string>("robotSimType");
  }
  else
  {
    robot_hw_sim_type_str_ = "lwr_hw/DefaultLWRHWsim";
    ROS_DEBUG_STREAM_NAMED("loadThread","Using default plugin for LWRHWsim (none specified in URDF/SDF)\""<<robot_hw_sim_type_str_<<"\"");
  }

  // Get the Gazebo simulation period
  ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

  // Decide the plugin control period
  if(sdf_->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if( control_period_ < gazebo_period )
    {
      ROS_ERROR_STREAM_NAMED("lwr_hw","Desired controller update period ("<<control_period_
        <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
    else if( control_period_ > gazebo_period )
    {
      ROS_WARN_STREAM_NAMED("lwr_hw","Desired controller update period ("<<control_period_
        <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
  }
  else
  {
    control_period_ = gazebo_period;
    ROS_DEBUG_STREAM_NAMED("lwr_hw","Control period not found in URDF/SDF, defaulting to Gazebo period of "
      << control_period_);
  }


  // Get parameters/settings for controllers from ROS param server
  model_nh_ = ros::NodeHandle(robot_namespace_);
  ROS_INFO_NAMED("lwr_hw", "Starting lwr_hw plugin in namespace: %s", robot_namespace_.c_str());

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  const std::string urdf_string = getURDF(robot_description_);
  if (!parseTransmissionsFromURDF(urdf_string))
  {
    ROS_ERROR_NAMED("lwr_hw", "Error parsing URDF in lwr_hw plugin, plugin not active.\n");
    return;
  }

  // Load the LWRHWsim abstraction to interface the controllers with the gazebo model
  try
  {
    robot_hw_sim_loader_.reset
      (new pluginlib::ClassLoader<lwr_hw::LWRHWsim>
        ("lwr_hw",
          "lwr_hw::LWRHWsim"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(!robot_hw_sim_->initSim(robot_namespace_, model_nh_, parent_model_, urdf_model_ptr, transmissions_))
    {
      ROS_FATAL_NAMED("lwr_hw","Could not initialize robot simulation interface");
      return;
    }

    // Create the controller manager
    ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_));

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin
      (boost::bind(&LWRHWSimPlugin::Update, this));

  }
  catch(pluginlib::LibraryLoadException &ex)
  {
    ROS_FATAL_STREAM_NAMED("lwr_hw","Failed to create robot simulation interface loader: "<<ex.what());
  }

  ROS_INFO_NAMED("lwr_hw", "Loaded lwr_hw.");
}

// Called by the world update start event
void LWRHWSimPlugin::Update()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    robot_hw_sim_->readSim(sim_time_ros, sim_period);

    // Compute the controller commands
    controller_manager_->update(sim_time_ros, sim_period);
  }

  // Update the gazebo model with the result of the controller
  // computation
  robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

// Called on world reset
void LWRHWSimPlugin::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = ros::Time();
  last_write_sim_time_ros_ = ros::Time();
}

// Get the URDF XML from the parameter server
std::string LWRHWSimPlugin::getURDF(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("lwr_hw", "lwr_hw plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("lwr_hw", "lwr_hw plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("lwr_hw", "Recieved urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool LWRHWSimPlugin::parseTransmissionsFromURDF(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);

  std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions_.begin();
  for(; it != transmissions_.end(); ) 
  {
    if (robot_namespace_.compare(it->robot_namespace_) != 0)
    {
      ROS_DEBUG_STREAM("lwr_hw_sim_plugin deleted transmission " << it->name_ << " because it is not in the same robotNamespace as this plugin. This might be normal in a multi-robot configuration though.");
      it = transmissions_.erase(it);
    }
    else
    {
      ++it;
    }
  }
  return true;
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LWRHWSimPlugin);
} // namespace
