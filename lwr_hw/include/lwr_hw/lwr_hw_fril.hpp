#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRIL remote hooks
#include <FastResearchInterface.h>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000
#define EOK 0

namespace lwr_hw
{

class LWRHWFRIL : public LWRHW
{

public:

  LWRHWFRIL() : LWRHW() {}
  ~LWRHWFRIL() {}

  void stop(){return;};
  void set_mode(){return;};

  void setInitFile(std::string init_file){init_file_ = init_file; file_set_ = true;};

  // Init, read, and write, with FRI hooks
  bool init()
  {
    if( !(file_set_) )
    {
      std::cout << "Did you forget to set the init file?" << std::endl
                << "You must do that before init()" << std::endl
                << "Exiting..." << std::endl;
      return false;
    }

    // construct a low-level lwr
    device_.reset( new FastResearchInterface( init_file_.c_str() ) );

    ResultValue	=	device_->StartRobot( FRI_CONTROL_POSITION );
    if (ResultValue != EOK)
    {
      std::cout << "An error occurred during starting up the robot...\n" << std::endl;
      return false;
    }

    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    float msrJntPos[n_joints_];
    float msrJntTrq[n_joints_];

    device_->GetMeasuredJointPositions( msrJntPos );
    device_->GetMeasuredJointTorques( msrJntTrq );

    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = (double)msrJntPos[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = (double)msrJntTrq[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
      joint_damping_[j] = joint_damping_command_[j];
    }
    return;
  }

  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

    // ensure the robot is powered and it is in control mode, almost like the isMachineOk() of Standford
    if ( device_->IsMachineOK() )
    {
      device_->WaitForKRCTick();

      switch (getControlStrategy())
      {

        case JOINT_POSITION:

          // Ensure the robot is in this mode
          if( (device_->GetCurrentControlScheme() == FRI_CONTROL_POSITION) )
          {
             float newJntPosition[n_joints_];
             for (int j = 0; j < n_joints_; j++)
             {
               newJntPosition[j] = (float)joint_position_command_[j];
             }
             device_->SetCommandedJointPositions(newJntPosition);
          }
          break;

        case CARTESIAN_IMPEDANCE:
          break;

         case JOINT_IMPEDANCE:

          // Ensure the robot is in this mode
          if( (device_->GetCurrentControlScheme() == FRI_CONTROL_JNT_IMP) )
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
              newJntPosition[j] = (float)joint_set_point_command_[j];
              newJntAddTorque[j] = (float)joint_effort_command_[j];
              newJntStiff[j] = (float)joint_stiffness_command_[j];
              newJntDamp[j] = (float)joint_damping_command_[j];
            }
            device_->SetCommandedJointStiffness(newJntStiff);
            device_->SetCommandedJointPositions(newJntPosition);
            device_->SetCommandedJointDamping(newJntDamp);
            device_->SetCommandedJointTorques(newJntAddTorque);
          }
          break;

         case GRAVITY_COMPENSATION:
           break;
       }
    }
    return;
  }

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
  {

    ResultValue	=	device_->StopRobot();
    if (ResultValue != EOK)
    {
        std::cout << "An error occurred during stopping the robot, couldn't switch mode...\n" << std::endl;
        return;
    }

    // at this point, we now that there is only one controller that ones to command joints
    ControlStrategy desired_strategy = JOINT_POSITION; // default

    desired_strategy = getNewControlStrategy(start_list,stop_list,desired_strategy);
    
    // only allow joint position and joint impedance control strategies, otherwise set the default (JOINT_POSITION) strategy
    if(desired_strategy != JOINT_POSITION && desired_strategy != JOINT_IMPEDANCE)
        desired_strategy = JOINT_POSITION;

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
      switch( desired_strategy )
      {
        case JOINT_POSITION:
          ResultValue = device_->StartRobot( FRI_CONTROL_POSITION );
          if (ResultValue != EOK)
          {
            std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_POSITION...\n" << std::endl;
            return;
          }
          break;
         case JOINT_IMPEDANCE:
          ResultValue = device_->StartRobot( FRI_CONTROL_JNT_IMP );
          if (ResultValue != EOK)
          {
            std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_IMPEDANCE...\n" << std::endl;
            return;
          }
          break;
      }

      // if sucess during the switch in FRI, set the ROS strategy
      setControlStrategy(desired_strategy);

      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
  }

private:

  // Parameters
  std::string init_file_;
  bool file_set_ = false;

  // low-level interface
  boost::shared_ptr<FastResearchInterface> device_;
  int ResultValue = 0;
};

}

#endif
