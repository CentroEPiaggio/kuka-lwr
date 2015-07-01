#include "lwr_hw/lwr_hw.h"

namespace lwr_hw
{

  void LWRHW::create()
  {
    // ALLOCATE MEMORY
    n_joints_ = 7; // safe magic number, the kuka lwr 4+ has 7 joints

    joint_names_.resize(n_joints_);
    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_stiffness_.resize(n_joints_);
    joint_damping_.resize(n_joints_);
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

    reset();

    // REGISTER INTERFACES
    // get the robot description
    robot_description_ = "/robot_description"; // default
    robot_namespace_ = "lwr";
    // Read urdf from ros parameter server then
    // setup actuators and mechanism control node.
    // This call will block if ROS is not properly initialized.
    const std::string urdf_string = getURDF(robot_description_);
    if (!parseTransmissionsFromURDF(urdf_string))
    {
      std::cout << "lwr_hw: " << "Error parsing URDF in lwr_hw plugin, plugin not active.\n" << std::endl;
      return;
    }
    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string) ? &urdf_model_ : NULL;
    registerInterfaces(urdf_model_ptr, transmissions_);

    // INIT KDL STUFF
    initKDLdescription(urdf_model_ptr);

    std::cout << "Succesfully created an abstract LWR 4+ ARM with interfaces" << std::endl;
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
      joint_damping_[j] = 0.0;

      joint_position_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;
      joint_effort_command_[j] = 0.0;
      joint_stiffness_command_[j] = 2500.0;
      joint_damping_command_[j] = 0.0;
    }

    current_strategy_ = JOINT_POSITION;

    return;
  }

  void LWRHW::registerInterfaces(const urdf::Model *const urdf_model, 
                     std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    // Initialize values
    for(unsigned int j=0; j < n_joints_; j++)
    {
      // Check that this transmission has one joint
      if(transmissions[j].joints_.size() == 0)
      {
        std::cout << "default_robot_hw_sim: " << "Transmission " << transmissions[j].name_
          << " has no associated joints." << std::endl;
        continue;
      }
      else if(transmissions[j].joints_.size() > 1)
      {
        std::cout << "default_robot_hw_sim: " << "Transmission " << transmissions[j].name_
          << " has more than one joint. Currently the default robot hardware simulation "
          << " interface only supports one." << std::endl;
        continue;
      }

      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

      if (joint_interfaces.empty() &&
          !(transmissions[j].actuators_.empty()) &&
          !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
      {
        // TODO: Deprecate HW interface specification in actuators in ROS J
        joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
        std::cout << "default_robot_hw_sim: " << "The <hardware_interface> element of tranmission " <<
          transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin." << std::endl;
      }
      if (joint_interfaces.empty())
      {
        std::cout << "default_robot_hw_sim: " << "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation." << std::endl;
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
      std::cout << "\x1B[37m" << "lwr_hw: " << "Loading joint '" << joint_names_[j]
        << "' of type '" << hardware_interface << "'" << "\x1B[0m" << std::endl;

      // Create joint state interface for all joints
      state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      std::cout << "CLAIM: j=" << j << " " << state_interface_.getClaims().size() << std::endl;

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

    std::cout << "CLAIMS: " << state_interface_.getClaims().size() << std::endl;
    registerInterface(&state_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&position_interface_);
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

  void LWRHW::enforceLimits(ros::Duration period)
  {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    vj_sat_interface_.enforceLimits(period);
    vj_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    sj_sat_interface_.enforceLimits(period);
    sj_limits_interface_.enforceLimits(period);
    dj_sat_interface_.enforceLimits(period);
    dj_limits_interface_.enforceLimits(period);
  }

  // Get the URDF XML from the parameter server
  std::string LWRHW::getURDF(std::string param_name) const
  {
    std::string urdf_string;

    ROS_INFO_ONCE_NAMED("lwr_hw", "lwr_hw is waiting for model"
      " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

    ros::param::get(robot_description_, urdf_string);

    ROS_INFO_ONCE_NAMED("lwr_hw", "Recieved urdf from param server, parsing...");

    return urdf_string;
  }

  // Get Transmissions from the URDF
  bool LWRHW::parseTransmissionsFromURDF(const std::string& urdf_string)
  {
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);

    std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions_.begin();
    for(; it != transmissions_.end(); ) 
    {
      if (robot_namespace_.compare(it->robot_namespace_) != 0)
      {
        std::cout << "lwr_hw deleted transmission " << it->name_ << " because it is not in the same robotNamespace as this plugin. This might be normal in a multi-robot configuration though." << std::endl;
        it = transmissions_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    return true;
  }

  // Init KDL stuff
  bool LWRHW::initKDLdescription(const urdf::Model *const urdf_model)
  {
    // KDL code to compute f_dyn(q)
      KDL::Tree kdl_tree;
      if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
      {
          ROS_ERROR("Failed to construct kdl tree");
          return false;
      }

      std::cout << "LWR kinematic successfully parsed with " 
                << kdl_tree.getNrOfJoints() 
                << " joints, and " 
                << kdl_tree.getNrOfJoints() 
                << " segments." << std::endl;;

      // Get the info from parameters
      std::string root_name;
      ros::param::get("root", root_name);
      if( root_name.empty() )
        root_name = kdl_tree.getRootSegment()->first; // default
      
      std::string tip_name;
      ros::param::get("tip", tip_name);
      if( root_name.empty() )
        tip_name = robot_namespace_ + std::string("_7_link"); ; // default

      // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
      gravity_ = KDL::Vector::Zero();
      gravity_(2) = -9.81;

      // Extract the chain from the tree
      if(!kdl_tree.getChain(root_name, tip_name, lwr_chain_))
      {
          ROS_ERROR("Failed to get KDL chain from tree: ");
          return false;
      }

      ROS_INFO("Number of segments: %d", lwr_chain_.getNrOfSegments());
      ROS_INFO("Number of joints in chain: %d", lwr_chain_.getNrOfJoints());

      f_dyn_solver_.reset(new KDL::ChainDynParam(lwr_chain_,gravity_));

      joint_position_kdl_ = KDL::JntArray(lwr_chain_.getNrOfJoints());
      gravity_effort_ = KDL::JntArray(lwr_chain_.getNrOfJoints());

      return true;
  }

}
