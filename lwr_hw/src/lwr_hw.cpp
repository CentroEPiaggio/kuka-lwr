#include "lwr_hw/lwr_hw.h"

namespace lwr_hw
{

// init resize variables
void LWRHW::init(int n_joints)
{
    n_joints_ = n_joints;
    joint_names_.resize(n_joints_);
    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_stiffness_.resize(n_joints_);
    joint_position_command_.resize(n_joints_);
    joint_velocity_command_.resize(n_joints_);
    joint_effort_command_.resize(n_joints_);
    joint_stiffness_command_.resize(n_joints_);
    joint_damping_command_.resize(n_joints_);

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);
    joint_lower_limits_stiffness_.resize(n_joints_);
    joint_upper_limits_stiffness_.resize(n_joints_);
    joint_effort_limits_.resize(n_joints_);

    return;
}

// reset values
void LWRHW::reset()
{
  for (int j = 0; j < n_joints_; ++j)
  {
    joint_position_[j] = 0.0;
    joint_position_prev_[j] = 0.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 0.0;
    joint_stiffness_[j] = 0.0;

    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;
    joint_effort_command_[j] = 0.0;
    joint_stiffness_command_[j] = 2500.0;
    joint_damping_command_[j] = 0.7;
  }

  return;
}

// Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
// retrieved from the urdf_model.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void LWRHW::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle_effort,
                         const hardware_interface::JointHandle& joint_handle_position,
                         const hardware_interface::JointHandle& joint_handle_velocity,
                         const hardware_interface::JointHandle& joint_handle_stiffness,
                         const urdf::Model *const urdf_model,
                         double *const lower_limit, double *const upper_limit, 
                         double *const lower_limit_stiffness, double *const upper_limit_stiffness,
                         double *const effort_limit)
{
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *lower_limit_stiffness = -std::numeric_limits<double>::max();
  *upper_limit_stiffness = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  joint_limits_interface::JointLimits limits_stiffness;
  bool has_limits = false;
  bool has_limits_stiffness = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
    const boost::shared_ptr<const urdf::Joint> urdf_joint_sitffness = urdf_model->getJoint(joint_name + std::string("_stiffness"));
    if (urdf_joint != NULL)
    {
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getJointLimits(urdf_joint_sitffness, limits_stiffness))
        has_limits_stiffness = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }

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
    const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle_effort(joint_handle_effort, limits, soft_limits);
    ej_limits_interface_.registerHandle(limits_handle_effort);
    const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle_position(joint_handle_position, limits, soft_limits);
    pj_limits_interface_.registerHandle(limits_handle_position);
    const joint_limits_interface::VelocityJointSoftLimitsHandle limits_handle_velocity(joint_handle_velocity, limits, soft_limits);
    vj_limits_interface_.registerHandle(limits_handle_velocity);

  }
  else
  {
    const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, limits);
    ej_sat_interface_.registerHandle(sat_handle_effort);
    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, limits);
    pj_sat_interface_.registerHandle(sat_handle_position);
    const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, limits);
    vj_sat_interface_.registerHandle(sat_handle_velocity);
  }


  if (!has_limits_stiffness)
    return;

  if (limits_stiffness.has_position_limits)
  {
    *lower_limit_stiffness = limits_stiffness.min_position;
    *upper_limit_stiffness = limits_stiffness.max_position;
  }

  const joint_limits_interface::PositionJointSaturationHandle sat_handle_stiffness(joint_handle_stiffness, limits_stiffness);
  sj_sat_interface_.registerHandle(sat_handle_stiffness);
}

}
