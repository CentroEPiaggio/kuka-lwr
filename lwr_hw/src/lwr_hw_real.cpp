
#include "lwr_hw/lwr_hw_real.h"

namespace lwr_hw
{

LWRHWreal::LWRHWreal(ros::NodeHandle nh) :
  nh_(nh)
{}

bool LWRHWreal::start()
{
  // get params or give default values
  nh_.param("port", port_, 49939);
  nh_.param("ip", hintToRemoteHost_, std::string("192.168.0.10") );

  // TODO: use transmission configuration to get names directly from the URDF model
  if( ros::param::get("joints", joint_names_) )
  {
    if( !(joint_names_.size()==LBR_MNJ) )
    {
      ROS_ERROR("This robot has 7 joints, you must specify 7 names for each one");
    } 
  }
  else
  {
    ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
    throw std::runtime_error("No joint name specification");
  }
  if( !(urdf_model_.initParam("/robot_description")) )
  {
    ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
    throw std::runtime_error("No URDF model available");
  }

  // construct a low-level lwr
  device_.reset( new friRemote( port_, const_cast<char*>(hintToRemoteHost_.c_str()) ) );

  // initialize FRI values
  lastQuality_ = FRI_QUALITY_BAD;
  lastCtrlScheme_ = FRI_CTRL_OTHER;

  // initialize and set to zero the state and command values
  init(LBR_MNJ);
  reset();

  // general joint to store information
  boost::shared_ptr<const urdf::Joint> joint;

  // create joint handles given the list
  for(int i = 0; i < LBR_MNJ; ++i)
  {
    ROS_INFO_STREAM("Handling joint: " << joint_names_[i]);

    // get current joint configuration
    joint = urdf_model_.getJoint(joint_names_[i]);
    if(!joint.get())
    {
      ROS_ERROR_STREAM("The specified joint "<< joint_names_[i] << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
      throw std::runtime_error("Wrong joint name specification");
    }

    // joint state handle
    hardware_interface::JointStateHandle state_handle(joint_names_[i],
        &joint_position_[i],
        &joint_velocity_[i],
        &joint_effort_[i]);

    state_interface_.registerHandle(state_handle);

    // effort command handle
    hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
          state_interface_.getHandle(joint_names_[i]),
          &joint_effort_command_[i]);
    effort_interface_.registerHandle(joint_handle_effort);

    // position command handle
    hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
          state_interface_.getHandle(joint_names_[i]),
          &joint_position_command_[i]);
    position_interface_.registerHandle(joint_handle_position);

    // stiffness command handle, registered in the position interface as well
    hardware_interface::JointHandle joint_handle_stiffness;
    joint_handle_stiffness = hardware_interface::JointHandle(hardware_interface::JointStateHandle(
                                                                 joint_names_[i]+std::string("_stiffness"),
                                                                 &joint_stiffness_[i], &joint_stiffness_[i], &joint_stiffness_[i]),
                                                     &joint_stiffness_command_[i]);
    position_interface_.registerHandle(joint_handle_stiffness);

    // velocity command handle, recall it is fake, there is no actual velocity interface
    hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
          state_interface_.getHandle(joint_names_[i]),
          &joint_velocity_command_[i]);

    registerJointLimits(joint_names_[i],
                        joint_handle_effort,
                        joint_handle_position,
                        joint_handle_velocity,
                        joint_handle_stiffness,
                        &urdf_model_,
                        &joint_lower_limits_[i],
                        &joint_upper_limits_[i],
                        &joint_lower_limits_stiffness_[i],
                        &joint_upper_limits_stiffness_[i],
                        &joint_effort_limits_[i]);
  }

  ROS_INFO("Register state and effort interfaces");

  // register ros-controls interfaces
  this->registerInterface(&state_interface_);
  this->registerInterface(&effort_interface_);
  this->registerInterface(&position_interface_);
  this->registerInterface(&velocity_interface_);

  // note that the velocity interface is not registrered, since the velocity command is computed within this implementation.

  std::cout << "Opening FRI Version " 
    << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
    << " Interface for LWR ROS server" << std::endl;

  ROS_INFO("Performing handshake to KRL");

  // perform some arbitrary handshake to KRL -- possible in monitor mode already
  // send to krl int a value
  device_->setToKRLInt(0,1);
  if ( device_->getQuality() >= FRI_QUALITY_OK)
  {
      // send a second marker
      device_->setToKRLInt(0,10);
  }

  //
  // just mirror the real value..
  //
  device_->setToKRLReal(0,device_->getFrmKRLReal(1));

  ROS_INFO_STREAM("LWR Status:\n" << device_->getMsrBuf().intf);

  device_->doDataExchange();
  ROS_INFO("Done handshake !");

  return true;
}

bool LWRHWreal::read(ros::Time time, ros::Duration period)
{
  // update the robot positions
  for (int j = 0; j < LBR_MNJ; j++)
  {
  	joint_position_prev_[j] = joint_position_[j];
    joint_position_[j] = device_->getMsrMsrJntPosition()[j];
    joint_effort_[j] = device_->getMsrJntTrq()[j];
    joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
    joint_stiffness_[j] = joint_stiffness_command_[j];
  }
  
  //this->device_->interface->doDataExchange();

  return true;
}

void LWRHWreal::write(ros::Time time, ros::Duration period)
{
  static int warning = 0;

  for (int j = 0; j < LBR_MNJ; j++)
  {
    // fake velocity command computed as:
    // (desired position - current position) / period, to avoid speed limit error
    joint_velocity_command_[j] = (joint_position_command_[j]-joint_position_[j])/period.toSec();
  }

  // enforce limits
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);

  // write to real robot
  float newJntPosition[LBR_MNJ];
  float newJntStiff[LBR_MNJ];
  float newJntDamp[LBR_MNJ];
  float newJntAddTorque[LBR_MNJ];

  if ( device_->isPowerOn() )
  { 
    // check control mode
    //if ( device_->getState() == FRI_STATE_CMD )
    //{
      // check control scheme
      if( device_->getCurrentControlScheme() == FRI_CTRL_JNT_IMP )
      {
        for (int i = 0; i < LBR_MNJ; i++)
        {
            newJntPosition[i] = joint_position_command_[i]; // zero for now
            newJntAddTorque[i] = joint_effort_command_[i]; // comes from the controllers
            newJntStiff[i] = joint_stiffness_command_[i]; // default values for now
            newJntDamp[i] = joint_damping_command_[i]; // default values for now
        }

        // only joint impedance control is performed, since it is the only one that provide access to the joint torque directly
        // note that stiffness and damping are 0, as well as the position, since only effort is allowed to be sent
        // the KRC adds the dynamic terms, such that if zero torque is sent, the robot apply torques necessary to mantain the robot in the current position
        // the only interface is effort, thus any other action you want to do, you have to compute the added torque and send it through a controller
        device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, true);
      }
    //}
  }

  // Stop request is issued from the other side
  /*
  if ( this->device_->interface->getFrmKRLInt(0) == -1)
  {
      ROS_INFO(" Stop request issued from the other side");
      this->stop();
  }*/

  // Quality change leads to output of statistics
  // for informational reasons
  //
  /*if ( this->device_->interface->getQuality() != this->device_->lastQuality )
  {
      ROS_INFO_STREAM("Quality change detected "<< this->device_->interface->getQuality()<< " \n");
      ROS_INFO_STREAM("" << this->device_->interface->getMsrBuf().intf);
      this->device_->lastQuality = this->device_->interface->getQuality();
  }*/

  // this is already done in the doJntImpedance Control setting to true the last flag
  // this->device_->interface->doDataExchange();

  return;
}

void LWRHWreal::stop()
{
  // TODO: decide whether to stop the FRI or just put to idle
  return;
}

void LWRHWreal::set_mode()
{
  // ToDo: just switch between monitor and command mode, no control strategies switch
  return;
}

} // namespace
