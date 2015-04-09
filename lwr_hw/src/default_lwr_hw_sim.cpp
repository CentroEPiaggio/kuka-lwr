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

#ifndef LWR_HW___DEFAULT_LWR_HW_SIM_H_
#define LWR_HW___DEFAULT_LWR_HW_SIM_H_

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// ros_control
#include <control_toolbox/pid.h>
#include <control_toolbox/filters.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl_parser/kdl_parser.hpp>

// lwr_hw_sim
#include "lwr_hw/lwr_hw_sim.h"

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace lwr_hw
{

class DefaultLWRHWsim : public lwr_hw::LWRHWsim
{
public:

  bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
    // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
    const ros::NodeHandle joint_limit_nh(model_nh, robot_namespace);

    // Resize vectors to our DOF, resize vectors, and reset to zero
    n_joints_ = transmissions.size();
    init(n_joints_);
    reset();

    // Initialize values
    for(unsigned int j=0; j < n_joints_; j++)
    {
      // Check that this transmission has one joint
      if(transmissions[j].joints_.size() == 0)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
          << " has no associated joints.");
        continue;
      }
      else if(transmissions[j].joints_.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
          << " has more than one joint. Currently the default robot hardware simulation "
          << " interface only supports one.");
        continue;
      }

      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

      if (joint_interfaces.empty() &&
          !(transmissions[j].actuators_.empty()) &&
          !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
      {
        // TODO: Deprecate HW interface specification in actuators in ROS J
        joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
          transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
      }
      if (joint_interfaces.empty())
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
        continue;
      }
      /*else if (joint_interfaces.size() > 1)
      {
        ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one.");
        continue;
      }
      */

      // Add data from transmission
      joint_names_.at(j) = transmissions.at(j).joints_.at(0).name_;

      const std::string& hardware_interface = joint_interfaces.front();

      // Debug
      ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
        << "' of type '" << hardware_interface << "'");

      // Create joint state interface for all joints
      state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle_effort;
      joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
      effort_interface_.registerHandle(joint_handle_effort);

      hardware_interface::JointHandle joint_handle_position;
      joint_handle_position = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
      position_interface_.registerHandle(joint_handle_position);

      // the stiffness is not actually a different joint, so the state handle is only used for handle
      hardware_interface::JointHandle joint_handle_stiffness;
      joint_handle_stiffness = hardware_interface::JointHandle(hardware_interface::JointStateHandle(
                                                                   joint_names_[j]+std::string("_stiffness"),
                                                                   &joint_stiffness_[j], &joint_stiffness_[j], &joint_stiffness_[j]),
                                                       &joint_stiffness_command_[j]);
      position_interface_.registerHandle(joint_handle_stiffness);
   
     // velocity command handle, recall it is fake, there is no actual velocity interface
      hardware_interface::JointHandle joint_handle_velocity;
      joint_handle_velocity = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
          &joint_velocity_command_[j]);

      // Get the gazebo joint that corresponds to the robot joint.
      //ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim", "Getting pointer to gazebo joint: "
      //  << joint_names_[j]);
      gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
      if (!joint)
      {
        ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
          << "\" which is not in the gazebo model.");
        return false;
      }
      sim_joints_.push_back(joint);

      registerJointLimits(joint_names_[j], 
                          joint_handle_effort, 
                          joint_handle_position,
                          joint_handle_velocity,
                          joint_handle_stiffness,
                          urdf_model, 
                          &joint_lower_limits_[j], &joint_upper_limits_[j],
                          &joint_lower_limits_stiffness_[j],
                          &joint_upper_limits_stiffness_[j],
                          &joint_effort_limits_[j]);

      /*if (joint_control_methods_[j] != EFFORT)
      {
        // Initialize the PID controller. If no PID gain values are found, use joint->SetPosition() or
        // joint->SetVelocity() to control the joint.
        const ros::NodeHandle nh(model_nh, robot_namespace + "/gazebo_ros_control/pid_gains/" +
                                 joint_names_[j]);
        if (pid_controllers_[j].init(nh, true))
        {
          switch (joint_control_methods_[j])
          {
            case POSITION:
              joint_control_methods_[j] = POSITION_PID;
              break;
            case VELOCITY:
              joint_control_methods_[j] = VELOCITY_PID;
              break;
          }
        }
        else
        {
          // joint->SetMaxForce() must be called if joint->SetPosition() or joint->SetVelocity() are
          // going to be called. joint->SetMaxForce() must *not* be called if joint->SetForce() is
          // going to be called.
          joint->SetMaxForce(0, joint_effort_limits_[j]);
        }
      }*/
    }

    // Register interfaces
    registerInterface(&state_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&position_interface_);

    // KDL code to compute f_dyn(q)
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    ROS_INFO("LWR kinematic successfully parsed with %d joints, and %d segments.",kdl_tree.getNrOfJoints(),kdl_tree.getNrOfJoints());

    // this is indepenedent of robot mounting, typically with positive z pointing up.-
    std::string root_name = kdl_tree.getRootSegment()->first; //std::string("world");

    // this could be parametrized to allow for different end-effectors as in the real robot.-
    std::string tip_name = robot_namespace + std::string("_7_link"); 

    // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Extract the chain from the tree
    if(!kdl_tree.getChain(root_name, tip_name, lwr_chain_))
    {
        ROS_ERROR("Failed to get KDL chain from tree: ");
        /*ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);
        */
        return false;
    }

    ROS_INFO("Number of segments: %d", lwr_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", lwr_chain_.getNrOfJoints());

    f_dyn_solver_.reset(new KDL::ChainDynParam(lwr_chain_,gravity_));

    joint_position_kdl_ = KDL::JntArray(lwr_chain_.getNrOfJoints());
    gravity_effort_ = KDL::JntArray(lwr_chain_.getNrOfJoints());

    return true;
  }

  void readSim(ros::Time time, ros::Duration period)
  {
    for(unsigned int j=0; j < n_joints_; j++)
    {

      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                              sim_joints_[j]->GetAngle(0).Radian());
      joint_position_kdl_(j) = joint_position_[j];
      // joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
      // derivate velocity as in the real hardware instead of reading it from simulation
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
    }
  }

  void writeSim(ros::Time time, ros::Duration period)
  {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    sj_sat_interface_.enforceLimits(period);
    sj_limits_interface_.enforceLimits(period);

    // compute the gracity term
    f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

    for(unsigned int j=0; j < n_joints_; j++)
    {
      // replicate the joint impedance control strategy
      // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
      double spring_effort = joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
      //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );

      const double effort = spring_effort + joint_effort_command_[j] + gravity_effort_(j);
      sim_joints_[j]->SetForce(0, effort);
    }
  }

  // KDL stuff to compute f_dyn in simulation
  KDL::Chain lwr_chain_;
  boost::scoped_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;
};

typedef boost::shared_ptr<DefaultLWRHWsim> DefaultLWRHWsimPtr;

}

PLUGINLIB_EXPORT_CLASS(lwr_hw::DefaultLWRHWsim, lwr_hw::LWRHWsim)

#endif
