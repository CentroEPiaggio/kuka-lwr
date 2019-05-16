#include "lwr_hw/lwr_hw.h"

namespace lwr_hw
{
  void LWRHW::create(std::string name, std::string urdf_string)
  {
    std::cout << "Creating a KUKA LWR 4+ called: " << name << std::endl;

    // SET NAME AND MODEL
    robot_namespace_ = name;
    urdf_string_ = urdf_string;

    // ALLOCATE MEMORY

    // JOINT NAMES ARE TAKEN FROM URDF NAME CONVENTION
    joint_names_.push_back( robot_namespace_ + std::string("_a1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a2_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_e1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a3_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a4_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a5_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a6_joint") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_x") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_y") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_z") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_X") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_Y") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_Z") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_A") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_B") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_C") );

    // VARIABLES
    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_stiffness_.resize(n_joints_);
    joint_damping_.resize(n_joints_);
    joint_position_command_.resize(n_joints_);
    joint_set_point_command_.resize(n_joints_);
    joint_velocity_command_.resize(n_joints_);
    joint_effort_command_.resize(n_joints_);
    joint_stiffness_command_.resize(n_joints_);
    joint_damping_command_.resize(n_joints_);
    cart_pos_.resize(12);
    cart_stiff_.resize(6);
    cart_damp_.resize(6);
    cart_wrench_.resize(6);
    cart_pos_command_.resize(12);
    cart_stiff_command_.resize(6);
    cart_damp_command_.resize(6);
    cart_wrench_command_.resize(6);

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);
    joint_lower_limits_stiffness_.resize(n_joints_);
    joint_upper_limits_stiffness_.resize(n_joints_);
    joint_upper_limits_damping_.resize(n_joints_);
    joint_lower_limits_damping_.resize(n_joints_);
    joint_effort_limits_.resize(n_joints_);

    // RESET VARIABLES
    reset();

    std::cout << "Parsing transmissions from the URDF..." << std::endl;

    // GET TRANSMISSIONS THAT BELONG TO THIS LWR 4+ ARM
    if (!parseTransmissionsFromURDF(urdf_string_))
    {
      std::cout << "lwr_hw: " << "Error parsing URDF in lwr_hw.\n" << std::endl;
      return;
    }

    std::cout << "Registering interfaces..." << std::endl;

    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : NULL;
    registerInterfaces(urdf_model_ptr, transmissions_);

    std::cout << "Initializing KDL variables..." << std::endl;

    // INIT KDL STUFF
    initKDLdescription(urdf_model_ptr);

    std::cout << "Succesfully created an abstract LWR 4+ ARM with interfaces to ROS control" << std::endl;
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
      joint_set_point_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;
      joint_effort_command_[j] = 0.0;
      joint_stiffness_command_[j] = 1000.0;
      joint_damping_command_[j] = 0.7;
    }

    for(int i=0; i < 12; ++i)
    {
      cart_pos_[i] = 0.0;
      cart_pos_command_[i] = 0.0;
    }
    cart_pos_[0] = 1.0;
    cart_pos_[5] = 1.0;
    cart_pos_[10] = 1.0;
    cart_pos_command_[0] = 1.0;
    cart_pos_command_[5] = 1.0;
    cart_pos_command_[10] = 1.0;
    for(int i=0; i < 3; i++)
    {
      cart_stiff_[i] = 0.0;
      cart_stiff_[i + 3] = 0.0;
      cart_damp_[i] = 0.0;
      cart_damp_[i + 3] = 0.0;
      cart_wrench_[i] = 0.0;
      cart_wrench_[i + 3] = 0.0;
      cart_stiff_command_[i] = 800;
      cart_stiff_command_[i + 3] = 50;
      cart_damp_command_[i] = 10;
      cart_damp_command_[i + 3] = 1;
      cart_wrench_command_[i] = 0.0;
      cart_wrench_command_[i + 3] = 0.0;
    }

    current_strategy_ = JOINT_POSITION;

    return;
  }

  void LWRHW::registerInterfaces(const urdf::Model *const urdf_model, 
                     std::vector<transmission_interface::TransmissionInfo> transmissions)
  {

    // Check that this transmission has one joint
    if( transmissions.empty() )
    {
      std::cout << "lwr_hw: " << "There are no transmission in this robot, all are non-driven joints? " 
        << std::endl;
      return;
    }

    // Initialize values
    for(int j=0; j < n_joints_; j++)
    {
      // Check that this transmission has one joint
      if(transmissions[j].joints_.size() == 0)
      {
        std::cout << "lwr_hw: " << "Transmission " << transmissions[j].name_
          << " has no associated joints." << std::endl;
        continue;
      }
      else if(transmissions[j].joints_.size() > 1)
      {
        std::cout << "lwr_hw: " << "Transmission " << transmissions[j].name_
          << " has more than one joint, and they can't be controlled simultaneously"
          << std::endl;
        continue;
      }

      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

      if( joint_interfaces.empty() )
      {
        std::cout << "lwr_hw: " << "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
          "You need to, otherwise the joint can't be controlled." << std::endl;
        continue;
      }

      const std::string& hardware_interface = joint_interfaces.front();

      // Debug
      std::cout << "\x1B[37m" << "lwr_hw: " << "Loading joint '" << joint_names_[j]
        << "' of type '" << hardware_interface << "'" << "\x1B[0m" << std::endl;

      // Create joint state interface for all joints
      state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle_effort;
      joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
      effort_interface_.registerHandle(joint_handle_effort);

      // To be able to read joint torques in the position cart interface
      position_cart_interface_.registerHandle(joint_handle_effort);

      hardware_interface::JointHandle joint_handle_position;
      joint_handle_position = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
      position_interface_.registerHandle(joint_handle_position);
      
      hardware_interface::JointHandle joint_handle_set_point;
      joint_handle_set_point = hardware_interface::JointHandle(hardware_interface::JointStateHandle(
                                                                   joint_names_[j]+std::string("_set_point"),
                                                                   &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]),
                                                       &joint_set_point_command_[j]);
      effort_interface_.registerHandle(joint_handle_set_point);

      // the stiffness is not actually a different joint, so the state handle is only used for handle
      hardware_interface::JointHandle joint_handle_stiffness;
      joint_handle_stiffness = hardware_interface::JointHandle(hardware_interface::JointStateHandle(
                                                                   joint_names_[j]+std::string("_stiffness"),
                                                                   &joint_stiffness_[j], &joint_stiffness_[j], &joint_stiffness_[j]),
                                                       &joint_stiffness_command_[j]);
      //position_interface_.registerHandle(joint_handle_stiffness);
      effort_interface_.registerHandle(joint_handle_stiffness);
      
      hardware_interface::JointHandle joint_handle_damping;
      joint_handle_damping = hardware_interface::JointHandle(hardware_interface::JointStateHandle(
                                                                    joint_names_[j]+std::string("_damping"),  
                                                                    &joint_damping_[j], &joint_damping_[j], &joint_damping_[j]),
                                                                    &joint_damping_command_[j]);
      effort_interface_.registerHandle(joint_handle_damping);
   
     // velocity command handle, recall it is fake, there is no actual velocity interface
      hardware_interface::JointHandle joint_handle_velocity;
      joint_handle_velocity = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
          &joint_velocity_command_[j]);

      registerJointLimits(joint_names_[j], 
                          joint_handle_effort, 
                          joint_handle_position,
                          joint_handle_velocity,
                          joint_handle_stiffness,
                          joint_handle_damping,
                          urdf_model, 
                          &joint_effort_limits_[j],
                          &joint_lower_limits_[j], &joint_upper_limits_[j],
                          &joint_lower_limits_stiffness_[j],
                          &joint_upper_limits_stiffness_[j],
                          &joint_lower_limits_damping_[j],
                          &joint_upper_limits_damping_[j]);
    }

    // Now for cart variables
    for(int j=0; j < 12; ++j)
    {
      cart_interface_.registerHandle(hardware_interface::CartesianStateHandle(
          cart_12_names_[j], &cart_pos_[j], &cart_stiff_[j], &cart_damp_[j]));
      hardware_interface::CartesianVariableHandle cart_pos_handle;
      cart_pos_handle = hardware_interface::CartesianVariableHandle(cart_interface_.getHandle(cart_12_names_[j]),
                                                       &cart_pos_command_[j]);
      position_cart_interface_.registerHandle(cart_pos_handle);
    }
    for(int j=0; j < 6; ++j)
    {
      cart_interface_.registerHandle(hardware_interface::CartesianStateHandle(
          cart_6_names_[j]+ std::string("_stiffness"), &cart_stiff_[j], &cart_damp_[j], &cart_wrench_[j]));
      hardware_interface::CartesianVariableHandle cart_stiff_handle;
      cart_stiff_handle = hardware_interface::CartesianVariableHandle(cart_interface_.getHandle(cart_6_names_[j] + std::string("_stiffness")),
                                                       &cart_stiff_command_[j]);
      position_cart_interface_.registerHandle(cart_stiff_handle);

      cart_interface_.registerHandle(hardware_interface::CartesianStateHandle(
          cart_6_names_[j]+ std::string("_damping"), &cart_stiff_[j], &cart_damp_[j], &cart_wrench_[j]));
      hardware_interface::CartesianVariableHandle cart_damp_handle;
      cart_damp_handle = hardware_interface::CartesianVariableHandle(cart_interface_.getHandle(cart_6_names_[j] + std::string("_damping")),
                                                       &cart_damp_command_[j]);
      position_cart_interface_.registerHandle(cart_damp_handle);

      cart_interface_.registerHandle(hardware_interface::CartesianStateHandle(
          cart_6_names_[j]+ std::string("_wrench"), &cart_stiff_[j], &cart_damp_[j], &cart_wrench_[j]));
      hardware_interface::CartesianVariableHandle cart_wrench_handle;
      cart_wrench_handle = hardware_interface::CartesianVariableHandle(cart_interface_.getHandle(cart_6_names_[j] + std::string("_wrench")),
                                                       &cart_wrench_command_[j]);
      position_cart_interface_.registerHandle(cart_wrench_handle);
    }

    // Register interfaces
    registerInterface(&state_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&position_interface_);
    registerInterface(&cart_interface_);
    registerInterface(&position_cart_interface_);
  }

  // Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
  // retrieved from the urdf_model.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  // TODO: register limits for cartesian variables

  void LWRHW::registerJointLimits(const std::string& joint_name, const hardware_interface::JointHandle& joint_handle_effort, const hardware_interface::JointHandle& joint_handle_position, const hardware_interface::JointHandle& joint_handle_velocity, const hardware_interface::JointHandle& joint_handle_stiffness,  const hardware_interface::JointHandle& joint_handle_damping, const urdf::Model*const urdf_model, double*const effort_limit, double*const lower_limit, double*const upper_limit, double*const lower_limit_stiffness, double*const upper_limit_stiffness, double*const lower_limit_damping, double*const upper_limit_damping)
{
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *lower_limit_stiffness = -std::numeric_limits<double>::max();
    *upper_limit_stiffness = std::numeric_limits<double>::max();
    *lower_limit_damping = -std::numeric_limits<double>::max();
    *upper_limit_damping = std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::JointLimits limits_stiffness;
    joint_limits_interface::JointLimits limits_damping;
    bool has_limits = false;
    bool has_limits_stiffness = false;
    bool has_limits_damping = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL)
    {
      const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
      const urdf::JointConstSharedPtr urdf_joint_sitffness = urdf_model->getJoint(joint_name + std::string("_stiffness"));
      const urdf::JointConstSharedPtr urdf_joint_damping = urdf_model->getJoint(joint_name + std::string("_damping"));
      if (urdf_joint != NULL)
      {
        // Get limits from the URDF file.
        if (joint_limits_interface::getJointLimits(urdf_joint, limits))
          has_limits = true;
        if (joint_limits_interface::getJointLimits(urdf_joint_sitffness, limits_stiffness))
          has_limits_stiffness = true;
        if (joint_limits_interface::getJointLimits(urdf_joint_damping, limits_damping))
            has_limits_damping = true;
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

    if (has_limits_stiffness)
    {

        if (limits_stiffness.has_position_limits)
        {
        *lower_limit_stiffness = limits_stiffness.min_position;
        *upper_limit_stiffness = limits_stiffness.max_position;
        }
        const joint_limits_interface::PositionJointSaturationHandle sat_handle_stiffness(joint_handle_stiffness, limits_stiffness);
        sj_sat_interface_.registerHandle(sat_handle_stiffness);
    }
    if (has_limits_damping)
    {
        
        if (limits_damping.has_position_limits)
        {
            *lower_limit_damping = limits_damping.min_position;
            *upper_limit_damping = limits_damping.max_position;
        }
        const joint_limits_interface::PositionJointSaturationHandle sat_handle_damping(joint_handle_damping, limits_damping);
        dj_sat_interface_.registerHandle(sat_handle_damping);
    }
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

  // Get Transmissions from the URDF
  bool LWRHW::parseTransmissionsFromURDF(const std::string& urdf_string)
  {
    std::vector<transmission_interface::TransmissionInfo> transmissions;

    // Only *standard* transmission_interface are parsed
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions);

    // Now iterate and save only transmission from this robot
    for (int j = 0; j < n_joints_; ++j)
    {
      // std::cout << "Check joint " << joint_names_[j] << std::endl;
      std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();
      for(; it != transmissions.end(); ++it)
      {
        // std::cout << "With transmission " << it->name_ << std::endl;
        if (joint_names_[j].compare(it->joints_[0].name_) == 0)
        {
          transmissions_.push_back( *it );
          // std::cout << "Found a match for transmission " << it->name_ << std::endl;
        }
      }
    }

    if( transmissions_.empty() )
      return false;

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
              << " segments." << std::endl;

    // Get the info from parameters
    std::string root_name;
    ros::param::get(std::string("/") + robot_namespace_ + std::string("/root"), root_name);
    if( root_name.empty() )
      root_name = kdl_tree.getRootSegment()->first; // default
    
    std::string tip_name;
    ros::param::get(std::string("/") + robot_namespace_ + std::string("/tip"), tip_name);
    if( tip_name.empty() )
      tip_name = robot_namespace_ + std::string("_7_link"); ; // default

    std::cout << "Using root: " << root_name << " and tip: " << tip_name << std::endl;

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

#if ROS_VERSION_MINIMUM(1,12,6)
    bool LWRHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const
    {
        int counter_position = 0, counter_effort = 0, counter_cartesian = 0;
        
        for ( std::list<hardware_interface::ControllerInfo>::const_iterator ci_it = start_list.begin(); ci_it != start_list.end(); ++ci_it )
        {
            for( std::vector<hardware_interface::InterfaceResources>::const_iterator it = ci_it->claimed_resources.begin(); it != ci_it->claimed_resources.end(); ++it)
            {
                // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
                if( it->hardware_interface.compare( std::string("hardware_interface::VelocityJointInterface") ) == 0 )
                {
                    std::cout << "The given controllers to start work on a velocity joint interface, and this robot does not have such an interface. "
                    << "The switch can't be done" << std::endl;
                    return false;
                }
                if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
                {
                    counter_position = 1;
                }
                else if( it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
                {
                    counter_effort = 1;
                }
                else if( it->hardware_interface.compare( std::string("hardware_interface::PositionCartesianInterface") ) == 0 )
                {
                    counter_cartesian = 1;
                }
            }
        }
        
        if( (counter_position+counter_effort+counter_cartesian)>1)
        {
            std::cout << "OOPS! Currently we are using the JointCommandInterface to switch mode, this is not strictly correct. " 
            << "This is temporary until a joint_mode_controller is available (so you can have different interfaces available in different modes)"
            << "Having said this, we do not support more than one controller that ones to act on any given JointCommandInterface"
            << "and we can't switch"
            << std::endl;
            return false;
        }
        
        return true;
    }
#else
  bool LWRHW::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const
  {
    int counter_position = 0, counter_effort = 0, counter_cartesian = 0;
    
    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
    {
      // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
      if( it->hardware_interface.compare( std::string("hardware_interface::VelocityJointInterface") ) == 0 )
      {
        std::cout << "The given controllers to start work on a velocity joint interface, and this robot does not have such an interface."
                  << "The switch can't be done" << std::endl;
        return false;
      }

      if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
      {
        // Debug
        // std::cout << "One controller wants to work on hardware_interface::PositionJointInterface" << std::endl;
        counter_position = 1;
      }
      else if( it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
      {
        // Debug
        // std::cout << "One controller wants to work on hardware_interface::EffortJointInterface" << std::endl;
        counter_effort = 1;
      }
      else if( it->hardware_interface.compare( std::string("hardware_interface::PositionCartesianInterface") ) == 0 )
      {
        // Debug
        // std::cout << "One controller wants to work on hardware_interface::PositionCartesianInterface" << std::endl;
        counter_cartesian = 1;
      }
      else
      {
        // Debug
        // std::cout << "This controller does not use any command interface, so it is only sensing, no problem" << std::endl;
      }
    }

    if( (counter_position+counter_effort+counter_cartesian)>1)
    {
      std::cout << "OOPS! Currently we are using the JointCommandInterface to switch mode, this is not strictly correct. " 
                << "This is temporary until a joint_mode_controller is available (so you can have different interfaces available in different modes)"
                << "Having said this, we do not support more than one controller that ones to act on any given JointCommandInterface"
                << "and we can't switch"
                << std::endl;
      return false;
    }

    return true;
  }
#endif

  void LWRHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
    // at this point, we now that there is only one controller that ones to command joints
    ControlStrategy desired_strategy = JOINT_POSITION; // default

    desired_strategy = getNewControlStrategy(start_list,stop_list,desired_strategy);

    for (int j = 0; j < n_joints_; ++j)
    {
      ///semantic Zero
      joint_position_command_[j] = joint_position_[j];
      joint_effort_command_[j] = 0.0;

      ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
      try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_position_command_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}
      try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_effort_command_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}

      ///reset joint_limit_interfaces
      pj_sat_interface_.reset();
      pj_limits_interface_.reset();
    }

    if(desired_strategy == getControlStrategy())
    {
      std::cout << "The ControlStrategy didn't change, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
      setControlStrategy(desired_strategy);
      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
  }

#if ROS_VERSION_MINIMUM(1,12,6)
    LWRHW::ControlStrategy LWRHW::getNewControlStrategy(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list, LWRHW::ControlStrategy default_control_strategy)
    {
        ControlStrategy desired_strategy = default_control_strategy;
        // NOTE that this allows to switch only based on the strategy of the first controller, but ROS kinetic would allow multiple interfaces and more
        bool strategy_found = false;
        
        for ( std::list<hardware_interface::ControllerInfo>::const_iterator ci_it = start_list.begin(); ci_it != start_list.end(); ++ci_it )
        {
            for( std::vector<hardware_interface::InterfaceResources>::const_iterator it = ci_it->claimed_resources.begin(); it != ci_it->claimed_resources.end(); ++it)
            {
                if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)" << std::endl;
                    desired_strategy = JOINT_POSITION;
                    strategy_found = true;
                    break;
                }
                else if( it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::EffortJointInterface (JOINT_IMPEDANCE)" << std::endl;
                    desired_strategy = JOINT_IMPEDANCE;
                    strategy_found = true;
                    break;
                }
                else if( it->hardware_interface.compare( std::string("hardware_interface::PositionCartesianInterface") ) == 0 )
                {
                    std::cout << "Request to switch to hardware_interface::PositionCartesianInterface (CARTESIAN_IMPEDANCE)" << std::endl;
                    desired_strategy = CARTESIAN_IMPEDANCE;
                    strategy_found = true;
                    break;
                }
            }
            if(strategy_found)
            {
                break;
            }
        }
        
        return desired_strategy;
    }
#else
  LWRHW::ControlStrategy LWRHW::getNewControlStrategy(const std::list< hardware_interface::ControllerInfo >& start_list, const std::list< hardware_interface::ControllerInfo >& stop_list, lwr_hw::LWRHW::ControlStrategy default_control_strategy)
  {
    ControlStrategy desired_strategy = default_control_strategy;
    
    // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
    {
        if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
        {
            std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)" << std::endl;
            desired_strategy = JOINT_POSITION;
            break;
        }
        else if( it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
        {
            std::cout << "Request to switch to hardware_interface::EffortJointInterface (JOINT_IMPEDANCE)" << std::endl;
            desired_strategy = JOINT_IMPEDANCE;
            break;
        }
        else if( it->hardware_interface.compare( std::string("hardware_interface::PositionCartesianInterface") ) == 0 )
        {
            std::cout << "Request to switch to hardware_interface::PositionCartesianInterface (CARTESIAN_IMPEDANCE)" << std::endl;
            desired_strategy = CARTESIAN_IMPEDANCE;
            break;
        }
    }
    
    return desired_strategy;
  }
#endif
  
}
