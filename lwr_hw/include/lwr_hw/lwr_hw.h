#ifndef LWR_HW__LWR_HW_H
#define LWR_HW__LWR_HW_H

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <control_toolbox/filters.h>
#include <urdf/model.h>

namespace lwr_hw
{

class LWRHW : public hardware_interface::RobotHW
{
protected:

  // Parameters
  urdf::Model urdf_model_;

  // hardware interfaces
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::EffortJointInterface effort_interface_;
  hardware_interface::PositionJointInterface position_interface_;

  // recalls that commands sent to this interface from outside are overwritten by our fake velocity command, and do nothing to the robot
  hardware_interface::VelocityJointInterface velocity_interface_;

  // joint limits interfaces
  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface   vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface   vj_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface sj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface sj_limits_interface_;

  // configuration
  int n_joints_;
  std::vector<std::string> joint_names_;

  // limits
  std::vector<double> 
  joint_lower_limits_,
  joint_upper_limits_,
  joint_effort_limits_ ,
  joint_lower_limits_stiffness_,
  joint_upper_limits_stiffness_;

  // state and commands
  std::vector<double>
  joint_position_,
  joint_position_prev_,
  joint_velocity_,
  joint_effort_,
  joint_stiffness_,
  joint_position_command_,
  joint_velocity_command_,
  joint_stiffness_command_,
  joint_damping_command_,
  joint_effort_command_;

  // NOTE:
  // joint_velocity_command is not really to command the kuka arm in velocity,
  // since it doesn't have an interface for that
  // this is used to avoid speed limit error in the kuka controller by
  // computing a fake velocity command using the received position command and
  // the current position, without smoothing.

  void init(int n_joints);
  void reset();
  void registerJointLimits(const std::string& joint_name,
                     const hardware_interface::JointHandle& joint_handle_effort,
                     const hardware_interface::JointHandle& joint_handle_position,
                     const hardware_interface::JointHandle& joint_handle_velocity,
                     const hardware_interface::JointHandle& joint_handle_stiffness,
                     const urdf::Model *const urdf_model,
                     double *const lower_limit, double *const upper_limit,
                     double *const lower_limit_stiffness, double *const upper_limit_stiffness,
                     double *const effort_limit);
}; // class

} // namespace

#endif
