#ifndef LWR_HW__LWR_HW_H
#define LWR_HW__LWR_HW_H

// boost
#include <boost/scoped_ptr.hpp>

// ROS headers
#include <std_msgs/Duration.h>
#include <urdf/model.h>

// ROS controls
#include <hardware_interface/robot_hw.h>
#include <cartesian_hardware_interface/cartesian_command_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <control_toolbox/filters.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl_parser/kdl_parser.hpp>

namespace lwr_hw
{

class LWRHW : public hardware_interface::RobotHW
{
public:

  LWRHW() {}
  virtual ~LWRHW() {}

  void create(std::string name, std::string urdf_string);

  // Strings
  std::string robot_namespace_;

  // Model
  std::string urdf_string_;
  urdf::Model urdf_model_;

  // control strategies
  // JOINT_POSITION -> strategy 10 -> triggered with PoitionJointInterface
  // CARTESIAN_IMPEDANCE -> strategy 20 (not implemented)
  // JOINT_IMPEDANCE -> strategy 30 -> triggered with (ToDo) ImpedanceJointInterface
// JOINT_EFFORT -> strategy 40 with special configuration, triggered with EffortJointInterface
// JOINT_STIFFNESS -> strategy 50 with special configuration, trigered with (ToDo) StiffnessJointInterface
// GRAVITY_COMPENSATION -> (does not exist in the real robot, achieved with low stiffness)
  // __Note__: StiffnessJointInterface + EffortJointInterface = ImpedanceJointInterface
  enum ControlStrategy {JOINT_POSITION = 10, CARTESIAN_IMPEDANCE = 20, JOINT_IMPEDANCE = 30, JOINT_EFFORT = 40, JOINT_STIFFNESS = 50, GRAVITY_COMPENSATION = 90};
#if ROS_VERSION_MINIMUM(1,12,6)
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const;
#else
  virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const;
#endif
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
  
  /**
   * @brief Function to get the control strategy based on the list of controllers to be started/stopped
   * 
   * @param start_list controllers to be started
   * @param stop_list controllers to be stopped
   * @param default_control_strategy value to use as a default return value
   * 
   * @return the output control strategy found based on the lists
   */
  static ControlStrategy getNewControlStrategy(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list, ControlStrategy default_control_strategy = JOINT_POSITION);

  // This functions must be implemented depending on the outlet (Real FRI/FRIL, Gazebo, etc.)
  virtual bool init() = 0;
  virtual void read(ros::Time time, ros::Duration period) = 0;
  virtual void write(ros::Time time, ros::Duration period) = 0;

  // get/set control method
  void setControlStrategy( ControlStrategy strategy){current_strategy_ = strategy;};
  ControlStrategy getControlStrategy(){ return current_strategy_;};

  // Hardware interfaces
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::EffortJointInterface effort_interface_;
  hardware_interface::PositionJointInterface position_interface_;
  hardware_interface::CartesianStateInterface cart_interface_;
  hardware_interface::PositionCartesianInterface position_cart_interface_;
  // hardware_interface::StiffnessJointInterface stiffness_interface_; // ToDo
  // hardware_interface::ImpedanceointInterface impedance_interface_; // ToDo

  ControlStrategy current_strategy_;

  // joint limits interfaces
  joint_limits_interface::EffortJointSaturationInterface     ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface     ej_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface   vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface   vj_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface   pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface   pj_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface   sj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface   sj_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface   dj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface   dj_limits_interface_;

  // Before write, you can use this function to enforce limits for all values
  void enforceLimits(ros::Duration period);

  // configuration
  int n_joints_ = 7; // safe magic number, the kuka lwr 4+ has 7 joints
  std::vector<std::string> joint_names_;
  std::vector<std::string> cart_12_names_;
  std::vector<std::string> cart_6_names_;

  // limits
  std::vector<double> 
  joint_lower_limits_,
  joint_upper_limits_,
  joint_effort_limits_ ,
  joint_lower_limits_stiffness_,
  joint_upper_limits_stiffness_,
  joint_lower_limits_damping_,
  joint_upper_limits_damping_;

  // state and commands
  std::vector<double>
  joint_position_,
  joint_position_prev_,
  joint_velocity_,
  joint_effort_,
  joint_stiffness_,
  joint_damping_,
  joint_position_command_,
  joint_set_point_command_,
  joint_velocity_command_,
  joint_stiffness_command_,
  joint_damping_command_,
  joint_effort_command_,
  cart_pos_,
  cart_stiff_,
  cart_damp_,
  cart_wrench_,
  cart_pos_command_,
  cart_stiff_command_,
  cart_damp_command_,
  cart_wrench_command_;

  // NOTE:
  // joint_velocity_command is not really to command the kuka arm in velocity,
  // since it doesn't have an interface for that
  // this is used to avoid speed limit error in the kuka controller by
  // computing a fake velocity command using the received position command and
  // the current position, without smoothing.

  // Set all members to default values
  void reset();

  // Transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // KDL stuff to compute ik, gravity term, etc.
  KDL::Chain lwr_chain_;
  boost::scoped_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

private:

  // Get Transmissions from the URDF
  bool parseTransmissionsFromURDF(const std::string& urdf_string);

  // Register all interfaces using
  void registerInterfaces(const urdf::Model *const urdf_model, 
                   std::vector<transmission_interface::TransmissionInfo> transmissions);

  // Initialize all KDL members
  bool initKDLdescription(const urdf::Model *const urdf_model);

  // Helper function to register limit interfaces
  void registerJointLimits(const std::string& joint_name, const hardware_interface::JointHandle& joint_handle_effort, const hardware_interface::JointHandle& joint_handle_position, const hardware_interface::JointHandle& joint_handle_velocity, const hardware_interface::JointHandle& joint_handle_stiffness, const hardware_interface::JointHandle& joint_handle_damping, const urdf::Model*const urdf_model, double*const effort_limit, double*const lower_limit, double*const upper_limit, double*const lower_limit_stiffness, double*const upper_limit_stiffness, double*const lower_limit_damping, double*const upper_limit_damping);

}; // class

} // namespace

#endif
