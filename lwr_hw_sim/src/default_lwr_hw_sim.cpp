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

#ifndef _GAZEBO_ROS_CONTROL___DEFAULT_LWR_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___DEFAULT_LWR_HW_SIM_H_

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

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl_parser/kdl_parser.hpp>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace gazebo_ros_control
{

class DefaultLWRHWSim : public gazebo_ros_control::RobotHWSim
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

    // Resize vectors to our DOF
    n_dof_ = transmissions.size();
    joint_names_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_lower_limits_.resize(n_dof_);
    joint_upper_limits_.resize(n_dof_);
    joint_effort_limits_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_position_prev_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_stiffness_command_.resize(n_dof_);
    joint_damping_command_.resize(n_dof_);

    // Initialize values
    for(unsigned int j=0; j < n_dof_; j++)
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
      joint_names_[j] = transmissions[j].joints_[0].name_;
      joint_position_[j] = 0.0;
      joint_position_prev_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 1.0;  // N/m for revolute joints
      joint_effort_command_[j] = 0.0;
      joint_position_command_[j] = 0.0;
      joint_stiffness_command_[j] = 3000.0;
      joint_damping_command_[j] = 0.0;

      const std::string& hardware_interface = joint_interfaces.front();

      // Debug
      ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
        << "' of type '" << hardware_interface << "'");

      // Create joint state interface for all joints
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle_effort;
      //if(hardware_interface == "EffortJointInterface")
      //{
        // Create effort joint interface
        //joint_control_methods_[j] = EFFORT;
      joint_handle_effort = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
      ej_interface_.registerHandle(joint_handle_effort);
      //}
      //else if(hardware_interface == "PositionJointInterface")
      //{
        // Create position joint interface
        //joint_control_methods_[j] = POSITION;
      hardware_interface::JointHandle joint_handle_position;
      joint_handle_position = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle_position);
      //}
      /*else
      {
        ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
          << hardware_interface );
        return false;
      }*/

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
                          joint_limit_nh, urdf_model,
                          &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
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
    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&pj_interface_);

    // KDL code to compute f_dyn(q)
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    ROS_INFO("LWR kinematic successfully parsed with %d joints, and %d segments.",kdl_tree.getNrOfJoints(),kdl_tree.getNrOfJoints());

    // this is indepenedent of robot mounting, typically with positive z pointing up.-
    std::string root_name = std::string("world"); 
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
    for(unsigned int j=0; j < n_dof_; j++)
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

    // compute the gracity term
    f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

    for(unsigned int j=0; j < n_dof_; j++)
    {
      // replicate the joint impedance control strategy
      // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
      double spring_effort = joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
      //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );

      const double effort = spring_effort + joint_effort_command_[j] + gravity_effort_(j);
      sim_joints_[j]->SetForce(0, effort);
    }
  }

private:
  // Methods used to control a joint.
  //enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle_effort,
                           const hardware_interface::JointHandle& joint_handle_position,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit)
  {
    *joint_type = urdf::Joint::REVOLUTE;
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL)
    {
      const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
      if (urdf_joint != NULL)
      {
        *joint_type = urdf_joint->type;
        // Get limits from the URDF file.
        if (joint_limits_interface::getJointLimits(urdf_joint, limits))
          has_limits = true;
        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
          has_soft_limits = true;
      }
    }
    // Get limits from the parameter server.
    if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
      has_limits = true;

    if (!has_limits)
      return;

    if (limits.has_position_limits)
    {
      *lower_limit = limits.min_position;
      *upper_limit = limits.max_position;
    }
    if (limits.has_effort_limits)
      *effort_limit = limits.max_effort;

    if (has_soft_limits)
    {
      const joint_limits_interface::EffortJointSoftLimitsHandle
        limits_handle_effort(joint_handle_effort, limits, soft_limits);
      ej_limits_interface_.registerHandle(limits_handle_effort);

      const joint_limits_interface::PositionJointSoftLimitsHandle
        limits_handle_position(joint_handle_position, limits, soft_limits);
      pj_limits_interface_.registerHandle(limits_handle_position);
    }
    else
    {
      const joint_limits_interface::EffortJointSaturationHandle
        sat_handle_effort(joint_handle_effort, limits);
      ej_sat_interface_.registerHandle(sat_handle_effort);

      const joint_limits_interface::PositionJointSaturationHandle
        sat_handle_position(joint_handle_position, limits);
      pj_sat_interface_.registerHandle(sat_handle_position);
    }
  }

  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<double> joint_position_;
  std::vector<double> joint_position_prev_; // to derivate joint velocity
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_stiffness_command_;
  std::vector<double> joint_damping_command_;

  // KDL stuff to compute f_dyn
  KDL::Chain lwr_chain_;
  boost::scoped_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

  std::vector<gazebo::physics::JointPtr> sim_joints_;
};

typedef boost::shared_ptr<DefaultLWRHWSim> DefaultLWRHWSimPtr;

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::DefaultLWRHWSim, gazebo_ros_control::RobotHWSim)

#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_LWR_HW_SIM_H_
