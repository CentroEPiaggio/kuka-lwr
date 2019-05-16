#ifndef LWR_HW____LWR_HW_SIM_H
#define LWR_HW____LWR_HW_SIM_H

// ROS
#include <angles/angles.h>
// #include <pluginlib/class_list_macros.h>

// Gazebo hook
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

namespace lwr_hw {

class LWRHWGazebo : public LWRHW
{
public:

  LWRHWGazebo() : LWRHW(), ts_(3.0), lastT_(0.0) {}
  ~LWRHWGazebo() {}

  void setParentModel(gazebo::physics::ModelPtr parent_model){parent_model_ = parent_model; parent_set_ = true;};

  // Init, read, and write, with Gazebo hooks
  bool init()
  {
    if( !(parent_set_) )
    {
      std::cout << "Did you forget to set the parent model?" << std::endl << "You must do that before init()" << std::endl << "Exiting..." << std::endl;
      return false;
    }

    gazebo::physics::JointPtr joint;
    for(int j=0; j < n_joints_; j++)
    {
      joint = parent_model_->GetJoint(joint_names_[j]);
      if (!joint)
      {
        std::cout << "This robot has a joint named \"" << joint_names_[j]
          << "\" which is not in the gazebo model." << std::endl;
        return false;
      }
      sim_joints_.push_back(joint);
    }

    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    for(int j=0; j < n_joints_; ++j)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                              sim_joints_[j]->Position(0));
      joint_position_kdl_(j) = joint_position_[j];
      // derivate velocity as in the real hardware instead of reading it from simulation
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j] - joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_effort_[j] = sim_joints_[j]->GetForce((int)(0));
      joint_stiffness_[j] = joint_stiffness_command_[j];
    }
  }

  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

    switch (getControlStrategy())
    {

      case JOINT_POSITION:
        for(int j=0; j < n_joints_; j++)
        {
          // according to the gazebo_ros_control plugin, this must *not* be called if SetForce is going to be called
          // but should be called when SetPostion is going to be called
          // so enable this when I find the SetMaxForce reset.
          // sim_joints_[j]->SetMaxForce(0, joint_effort_limits_[j]);
#if GAZEBO_MAJOR_VERSION >= 4
          sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#else
          sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
#endif
        }
        break;

      case CARTESIAN_IMPEDANCE:
        if(time-lastT_ < ts_)
            break;
        lastT_ = time;
        ROS_WARN("CARTESIAN IMPEDANCE NOT AVAILABLE IN GAZEBO, PRINTING THE COMMANDED VALUES:");
        std::cout << "Notice that this printing is done only every " << ts_.toSec() << " seconds: to change this, change ts_ in lwr_hw_gazebo.hpp..." << std::endl;
        std::cout << "cart_pos_command_ = | ";
        for(int i=0; i < 12; ++i)
            std::cout << cart_pos_command_[i] << " | ";
        std::cout << std::endl << "cart_stiff_command_ = | ";
        for(int i=0; i < 6; i++)
            std::cout << cart_stiff_command_[i] << " | ";
        std::cout << std::endl << "cart_damp_command_ = | ";
        for(int i=0; i < 6; i++)
            std::cout << cart_damp_command_[i] << " | ";
        std::cout << std::endl << "cart_wrench_command_ = | ";
        for(int i=0; i < 6; i++)
            std::cout << cart_wrench_command_[i] << " | ";
        std::cout << std::endl << "Here, the call to doCartesianImpedanceControl() is done" << std::endl;
        break;

      case JOINT_IMPEDANCE:
        // compute the gracity term
        f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);
        
        for(int j=0; j < n_joints_; j++)
        {
          // replicate the joint impedance control strategy
          // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
          const double stiffness_effort = 0.0;//10.0*( joint_position_command_[j] - joint_position_[j] ); // joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
          //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );
          const double effort = stiffness_effort + joint_effort_command_[j] + gravity_effort_(j);
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case GRAVITY_COMPENSATION:
        ROS_WARN("CARTESIAN IMPEDANCE NOT IMPLEMENTED");
        break;
    }
  }

private:

  // Gazebo stuff
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::ModelPtr parent_model_;
  bool parent_set_ = false;
  ros::Duration ts_;
  ros::Time lastT_;

};

}

#endif