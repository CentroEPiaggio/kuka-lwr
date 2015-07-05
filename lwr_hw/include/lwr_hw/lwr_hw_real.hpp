#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRI remote hooks
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include "fri/friudp.h"
#include "fri/friremote.h"

// ToDo: add timeouts to all sync-while's to KRL since the UDP connection might be lost and we will know

namespace lwr_hw
{

class LWRHWreal : public LWRHW
{

public:

  LWRHWreal() : LWRHW() {}
  ~LWRHWreal() {}

  void stop(){return;};
  void set_mode(){return;};

  void setPort(int port){port_ = port; port_set_ = true;};
  void setIP(std::string hintToRemoteHost){hintToRemoteHost_ = hintToRemoteHost; ip_set_ = true;};

  // Init, read, and write, with FRI hooks
  bool init()
  {
    if( !(port_set_) || !(ip_set_) )
    {
      std::cout << "Did you forget to set the port/ip?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
      return false;
    }

    // construct a low-level lwr
    device_.reset( new friRemote( port_, const_cast<char*>(hintToRemoteHost_.c_str()) ) );

    // initialize FRI values
    lastQuality_ = FRI_QUALITY_BAD;
    lastCtrlScheme_ = FRI_CTRL_OTHER;

    std::cout << "Opening FRI Version " 
      << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
      << " Interface for LWR ROS server" << std::endl;

    std::cout << "Performing handshake with the KRC unit..." << std::endl;

    // if off, wait for monitor mode to avoid loosing UDP packages
    if( device_->getState() == FRI_STATE_OFF )
    {
      while( device_->getState() != FRI_STATE_MON )
      {
        std::cout << "Please, start the KRL script now." << std::endl;
        usleep(1000000);
      }
    }

    std::cout << "Performing handshake with the KRC unit..." << std::endl;

    // salute KRL
    device_->setToKRLInt(15,1);
    // be polite and wait for KRL to salute back
    while( device_->getFrmKRLInt(15) == 1 )
    {
      device_->doDataExchange();
    }

    std::cout << "Done handshake." << std::endl;
    
    // wait for good quality
    while ( device_->getQuality() >= FRI_QUALITY_OK) {}

    // wait for FRI to start
    while ( device_->getState() != FRI_STATE_CMD) {}

    // debug
    // std::cout << "LWR Status:\n" << device_->getMsrBuf().intf << std::endl;

    std::cout << "FRI has been started!" << std::endl;
    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = device_->getMsrMsrJntPosition()[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = device_->getMsrJntTrq()[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
    }
    return;
  }

  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

    // ensure the robot is powered and it is in control mode, almost like the isMachineOk() of Standford
    if ( device_->isPowerOn() && (device_->getState() == FRI_STATE_CMD) )
    { 
      switch (getControlStrategy())
      {

        case JOINT_POSITION:

          // Ensure the robot is in this mode
          if( (device_->getCurrentControlScheme() == FRI_CTRL_POSITION) )
          {
            float newJntPosition[n_joints_];

            for (unsigned int j = 0; j < n_joints_; j++)
            {
              newJntPosition[j] = joint_position_command_[j]; 
            }
            device_->doPositionControl(newJntPosition, true);
          }
          break;

        case CARTESIAN_IMPEDANCE:
          ROS_WARN("CARTESIAN IMPEDANCE NOT IMPLEMENTED");
          break;

        case JOINT_IMPEDANCE:

          // Ensure the robot is in this mode
          if( (device_->getCurrentControlScheme() == FRI_CTRL_JNT_IMP) )
          {
            float newJntPosition[n_joints_];
            float newJntStiff[n_joints_];
            float newJntDamp[n_joints_];
            float newJntAddTorque[n_joints_];

            // WHEN THE URDF MODEL IS PRECISE
            // 1. compute the gracity term
            // f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

            // 2. read gravity term from FRI and add it with opposite sign and add the URDF gravity term
            // newJntAddTorque = gravity_effort_  - device_->getF_DYN??
            
            for(int j=0; j < n_joints_; j++)
            {
              newJntPosition[j] = joint_position_command_[j];
              newJntAddTorque[j] = joint_effort_command_[j];
              newJntStiff[j] = joint_stiffness_command_[j];
              newJntDamp[j] = joint_damping_command_[j];
            }
            device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, true);
          }
          break;

        case GRAVITY_COMPENSATION:
          if( device_->getCurrentControlScheme() == FRI_CTRL_OTHER )
          {
            // just read status to keep FRI alive
            device_->doDataExchange();
          }
          break;
      }
    }
    return;
  }

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
    // at this point, we now that there is only one controller that ones to command joints
    ControlStrategy desired_strategy = JOINT_POSITION; // default

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
    }

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
      std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
      setControlStrategy(desired_strategy);
      
      // trigger the KRL with the new stragety value
      device_->setToKRLInt(0, desired_strategy);
      device_->doDataExchange();

      // wait until friStop() is called
      while ( device_->getFrmKRLInt(0) != 0) {}

      // wait for good quality comm again
      while ( device_->getQuality() >= FRI_QUALITY_OK) {}

      // wait for FRI to start again before exiting the switch
      while ( device_->getState() != FRI_STATE_CMD) {}

      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
  }

private:

  // Parameters
  int port_;
  bool port_set_ = false;
  std::string hintToRemoteHost_;
  bool ip_set_ = false;

  // low-level interface
  boost::shared_ptr<friRemote> device_;

  // FRI values
  FRI_QUALITY lastQuality_;
  FRI_CTRL lastCtrlScheme_;

};

}

#endif
