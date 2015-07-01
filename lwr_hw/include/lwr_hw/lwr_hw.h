#ifndef LWR_HW__LWR_HW_H
#define LWR_HW__LWR_HW_H

// boost
#include <boost/scoped_ptr.hpp>

// ROS headers
#include <std_msgs/Duration.h>
#include <urdf/model.h>

// ROS controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
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

  virtual ~LWRHW() {}

  void create();

  // control strategies
  // position strategy 10 -> triggered with PoitionJointInterface
  // cartesian impedance -> strategy 20 (not implemented)
  // impedance strategy 30 -> triggered with EffortJointInterface
  // gravity compensation mode -> (not implemented, achieved with low stiffness)
  enum ControlStrategy {JOINT_POSITION, CARTESIAN_IMPEDANCE, JOINT_IMPEDANCE, GRAVITY_COMPENSATION};

  // This functions must be implemented depending on the outlet (Real FRI/FRIL, Gazebo, etc.)
  virtual bool init();
  virtual void read(ros::Time time, ros::Duration period);
  virtual void write(ros::Time time, ros::Duration period);

  // Before write, you can use this function to enforce limits for all values
  void enforceLimits(ros::Duration period);

  // Set all members to default values
  void reset();

  // get/set control method
  bool setControlStrategy( ControlStrategy current_strategy){current_strategy_ = current_strategy;}; // CHECK CONFLICT
  ControlStrategy getControlStrategy(){ return current_strategy_;};

  // Strings
  std::string robot_namespace_;
  std::string robot_description_;

  // Model
  urdf::Model urdf_model_;

  // Hardware interfaces
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::EffortJointInterface effort_interface_;
  hardware_interface::PositionJointInterface position_interface_;

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

  // configuration
  int n_joints_;
  std::vector<std::string> joint_names_;

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

  // Transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // KDL stuff to compute ik, gravity term, etc.
  KDL::Chain lwr_chain_;
  boost::scoped_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const;

  // Get Transmissions from the URDF
  bool parseTransmissionsFromURDF(const std::string& urdf_string);

  // Register all interfaces using
  void registerInterfaces(const urdf::Model *const urdf_model, 
                   std::vector<transmission_interface::TransmissionInfo> transmissions);

  // Initialize all KDL members
  bool initKDLdescription(const urdf::Model *const urdf_model);

  // Helper function to register limit interfaces
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
