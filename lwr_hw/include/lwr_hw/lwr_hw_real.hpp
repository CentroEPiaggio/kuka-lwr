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

namespace lwr_hw
{

class LWRHWreal : public LWRHW
{

public:

  ~LWRHWreal() {}

  void stop(){return;};
  void set_mode(){return;};

  void setPort(int port){port_ = port; port_set_ = true;};
  void setIP(std::string hintToRemoteHost){hintToRemoteHost_ = hintToRemoteHost; ip_set_ = true;};

  // Init, read, and write, with FRI hooks
  bool init()
  {
    create();

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

    std::cout << "Performing handshake to KRL" << std::endl;

    // perform some arbitrary handshake to KRL -- possible in monitor mode already
    // send to krl int a value
    device_->setToKRLInt(0,1);
    if ( device_->getQuality() >= FRI_QUALITY_OK)
    {
        // send a second marker
        device_->setToKRLInt(0,10);
    }

    // just mirror the real value..
    device_->setToKRLReal(0,device_->getFrmKRLReal(1));

    std::cout << "LWR Status:\n" << device_->getMsrBuf().intf << std::endl;

    device_->doDataExchange();
    std::cout << "Done handshake !" << std::endl;

    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = device_->getMsrMsrJntPosition()[j];
      joint_effort_[j] = device_->getMsrJntTrq()[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
    }
    return;
  }

  void write(ros::Time time, ros::Duration period)
  {
    // fake velocity command computed as:
    // (desired position - current position) / period, to avoid speed limit error
    for (int j = 0; j < n_joints_; j++)
    {
      joint_velocity_command_[j] = (joint_position_command_[j]-joint_position_[j])/period.toSec();
    }

    // enforce limits
    enforceLimits(period);

    // write to real robot
    float newJntPosition[n_joints_];
    float newJntStiff[n_joints_];
    float newJntDamp[n_joints_];
    float newJntAddTorque[n_joints_];

    if ( device_->isPowerOn() )
    { 
      // check control mode
      //if ( device_->getState() == FRI_STATE_CMD )
      //{
        // check control strategy
        if( device_->getCurrentControlScheme() == FRI_CTRL_JNT_IMP )
        {
          for (int i = 0; i < n_joints_; i++)
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
        else if( device_->getCurrentControlScheme() == FRI_CTRL_POSITION )
        {
          for (int i = 0; i < n_joints_; i++)
          {
              newJntPosition[i] = joint_position_command_[i]; 
          }

          // only joint impedance control is performed, since it is the only one that provide access to the joint torque directly
          // note that stiffness and damping are 0, as well as the position, since only effort is allowed to be sent
          // the KRC adds the dynamic terms, such that if zero torque is sent, the robot apply torques necessary to mantain the robot in the current position
          // the only interface is effort, thus any other action you want to do, you have to compute the added torque and send it through a controller
          device_->doPositionControl(newJntPosition, true);
        } 
        else if( device_->getCurrentControlScheme() == FRI_CTRL_OTHER ) // Gravity compensation: just read status, but we have to keep FRI alive
        {
          device_->doDataExchange();
        }
      //}
    }
    return;
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