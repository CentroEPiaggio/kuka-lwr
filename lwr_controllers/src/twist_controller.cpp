#include <lwr_controllers/twist_controller.h>
#include <pluginlib/class_list_macros.h>
#include <utils/pseudo_inversion.h>
#include "kdl/chainfksolvervel_recursive.hpp"

#define DEBUG   1                       // prints out additional info

namespace lwr_controllers {

  // DEFAULT CONSTRUCTOR
  TwistController::TwistController(){
    // Nothing to do
  }

  // DEFAULT DESTRUCTOR
  TwistController::~TwistController(){
    // Nothing to do
  }

  // INITIALIZING FUNCTION
  bool TwistController::init(hardware_interface::PositionJointInterface *robot,
    ros::NodeHandle &n){
    // Calling the Initializing function of KinematicChainControllerBase
    if(!(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n))){
        ROS_ERROR("Couldn't initilize TwistController.");
        return false;
    }

    // Resetting the solvers
    jnt_to_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resizing vectors and jacobian
    joint_states_.resize(kdl_chain_.getNrOfJoints());
    joint_states_old_.resize(kdl_chain_.getNrOfJoints());
    joint_states_real_.resize(kdl_chain_.getNrOfJoints());
    joint_vel_comm_.resize(kdl_chain_.getNrOfJoints());
    joint_comm_.resize(kdl_chain_.getNrOfJoints());
    jacobian_.resize(kdl_chain_.getNrOfJoints());

    // Initializing the cmd_flag_
    cmd_flag_ = 0;

    // Initializing subscriber to command topic
    sub_command_ = n.subscribe<geometry_msgs::Twist>("command", 1,
      &TwistController::command, this);

    return true;
  }

  // STARTING FUNCTION
  void TwistController::starting(const ros::Time& time){
    // Setting all twists to zero
    twist_des_ = KDL::Twist::Zero();
    twist_des_new_ = KDL::Twist::Zero();
    twist_ach_ = KDL::Twist::Zero();
    twist_des_old_ = KDL::Twist::Zero();
    twist_error_ = KDL::Twist::Zero();
    twist_real_ = KDL::Twist::Zero();
    twist_error_real_ = KDL::Twist::Zero();
    twist_check_ = KDL::Twist::Zero();

    // Reading the joint states (position and velocity) and setting command
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
      joint_states_real_(i) = joint_handles_[i].getPosition();
      joint_comm_(i) = joint_states_real_(i);
      joint_handles_[i].setCommand(joint_comm_(i));
    }

    // Copying into the "non real" joint variables
    joint_states_ = joint_states_real_;
    joint_states_old_ = joint_states_;

    // Computing the initial cartesian position
    jnt_to_pos_solver_->JntToCart(joint_states_, P_ee_);
    P_ee_old_ = P_ee_;
    P_ee_real_ = P_ee_;
    P_ee_real_old_ = P_ee_;
  }

  // UPDATING FUNCTION
  void TwistController::update(const ros::Time& time,
    const ros::Duration& period){
    // THE CODE INSIDE NEXT IF EXECUTED ONLY IF twist_des_ != 0
    if(cmd_flag_){
      // Reading the joint states (position and velocity)
      for(unsigned int i = 0; i < joint_handles_.size(); i++){
        joint_states_real_(i) = joint_handles_[i].getPosition();
      }

      // Debug messages
      ROS_DEBUG_STREAM("The previous joint_comm_  = " << this->joint_comm_.data << ".");
      ROS_DEBUG_STREAM("The current joint_states_ = " << this->joint_states_.data << ".");
      ROS_DEBUG_STREAM("The current dt = " << period.toSec() << ".");

      // Forward computing current ee pose and then twist error
      jnt_to_pos_solver_->JntToCart(joint_states_, P_ee_);
      jnt_to_pos_solver_->JntToCart(joint_states_real_, P_ee_real_);
      twist_ach_ = computeTwistFromFrames(P_ee_, P_ee_old_, period);
      twist_real_ = computeTwistFromFrames(P_ee_real_, P_ee_real_old_, period);
      twist_error_ = twist_des_old_ - twist_ach_;
      twist_error_real_ = twist_des_old_ - twist_real_;

      // Debug messages
      ROS_DEBUG_STREAM("The wanted twist was = \n" << twist_des_old_.vel.data[0] << "\n" << twist_des_old_.vel.data[1] << "\n"
        << twist_des_old_.vel.data[2] << "\n" << twist_des_old_.rot.data[0] << "\n" << twist_des_old_.rot.data[1] << "\n"
        << twist_des_old_.rot.data[2] << ".");
      ROS_DEBUG_STREAM("The achieved twist is = \n" << twist_ach_.vel.data[0] << "\n" << twist_ach_.vel.data[1] << "\n"
        << twist_ach_.vel.data[2] << "\n" << twist_ach_.rot.data[0] << "\n" << twist_ach_.rot.data[1] << "\n"
        << twist_ach_.rot.data[2] << ".");
      ROS_DEBUG_STREAM("The twist error is = \n" << twist_error_.vel.data[0] << "\n" << twist_error_.vel.data[1] << "\n"
        << twist_error_.vel.data[2] << "\n" << twist_error_.rot.data[0] << "\n" << twist_error_.rot.data[1] << "\n"
        << twist_error_.rot.data[2] << ".");
      ROS_DEBUG_STREAM("The real twist error is = \n" << twist_error_real_.vel.data[0] << "\n" << twist_error_real_.vel.data[1] << "\n"
        << twist_error_real_.vel.data[2] << "\n" << twist_error_real_.rot.data[0] << "\n" << twist_error_real_.rot.data[1] << "\n"
        << twist_error_real_.rot.data[2] << ".");
      ROS_DEBUG_STREAM("The joint position error is = \n" << joint_states_real_.data - joint_states_.data << ".");

      // Adding up the previos error in the current desired twist
      // (THE twist_error_ CAN BE USED: not sure if this brings to a better behavior)
      twist_des_new_ = twist_des_ + twist_error_;
      // twist_des_new_ = twist_des_;

      // Updating old twists
      twist_des_old_ = twist_des_;

      // Computing the current jacobian
      jnt_to_jac_solver_->JntToJac(joint_states_, jacobian_);

      // Computing the pseudo inverse of the jacobian
      pseudo_inverse(jacobian_.data, jacobian_pinv_, true);    // false in order to remove damping in pinv

      // Compute the joint velocities using jacobian pseudo inverse
      for(unsigned int i = 0; i < jacobian_pinv_.rows(); i++){
        joint_vel_comm_(i) = 0.0;
        for(unsigned int j = 0; j < jacobian_pinv_.cols(); j++){
          joint_vel_comm_(i) += jacobian_pinv_(i, j) * twist_des_new_(j);
        }
      }

      // Checking if the desired twist will be achieved by the joint_vel_comm_
      for(unsigned int i = 0; i < jacobian_.data.rows(); i++){
        twist_check_(i) = twist_des_new_(i);
        for(unsigned int j = 0; j < jacobian_.data.cols(); j++){
          twist_check_(i) -= jacobian_.data(i, j) * joint_vel_comm_(j);
        }
      }
      ROS_DEBUG_STREAM("The pseudo inversion gave an error in twist of \n" << twist_check_.vel.data[0] << "\n" << twist_check_.vel.data[1] << "\n"
        << twist_check_.vel.data[2] << "\n" << twist_check_.rot.data[0] << "\n" << twist_check_.rot.data[1] << "\n"
        << twist_check_.rot.data[2] << ".");

      // Setting command to current position
      joint_comm_ = joint_states_;

      // Integrating joint_vel_comm_ to find joint_comm_ (forward Euler)
      for(unsigned int i = 0; i < joint_handles_.size(); i++){
        joint_comm_(i) += joint_vel_comm_(i) * period.toSec();
      }

      // Saturating with joint limits
      for(unsigned int i = 0; i < joint_handles_.size(); i++){
        if(joint_comm_(i) < joint_limits_.min(i)){
          ROS_WARN_STREAM("MIN SAT: The " << i << "th joint command " << joint_comm_(i) << " has been saturated to " << joint_limits_.min(i) << ". \n");
          joint_comm_(i) = joint_limits_.min(i);
        }
        if(joint_comm_(i) > joint_limits_.max(i)){
          ROS_WARN_STREAM("MAX SAT: The " << i << "th joint command " << joint_comm_(i) << " has been saturated to " << joint_limits_.max(i) << ". \n");
          joint_comm_(i) = joint_limits_.max(i);
        }
      }

      // Memorizing current joint_states_ to old and updating joint_states_
      joint_states_old_ = joint_states_;
      joint_states_ = joint_comm_;

      // Memorizing old ee pose
      P_ee_old_ = P_ee_;
      P_ee_real_old_ = P_ee_real_;

      // Debug to check execution frequency
      ROS_DEBUG_STREAM("TwistController: Executing Twist Command!!!");
    }

    // Debug message
    ROS_DEBUG_STREAM("The current joint command = " << this->joint_comm_.data << ".");

    // Once we have the torques, setting the effort
    for(unsigned int i = 0; i < joint_handles_.size(); i++){
      joint_handles_[i].setCommand(joint_comm_(i));
    }
  }

  // COMMAND TOPIC CALLBACK FUNCTION
  void TwistController::command(const geometry_msgs::Twist::ConstPtr &msg){
    // Converting and saving msg to twist_des_
    twist_des_.vel(0) = msg->linear.x;
    twist_des_.vel(1) = msg->linear.y;
    twist_des_.vel(2) = msg->linear.z;
    twist_des_.rot(0) = msg->angular.x;
    twist_des_.rot(1) = msg->angular.y;
    twist_des_.rot(2) = msg->angular.z;

    // Setting cmd_flag_ according to the twist
    if(KDL::Equal(twist_des_, KDL::Twist::Zero(), 0.0001)) cmd_flag_ = 0;
    else cmd_flag_ = 1;
  }

  // Auxiliary functions
  KDL::Twist TwistController::computeTwistFromFrames(KDL::Frame Pose_1, KDL::Frame Pose_2, ros::Duration dt){
    // Computing linear velocity
    KDL::Vector lin_vel = (Pose_1.p - Pose_2.p) / dt.toSec();

    // Computing angular velocity (Math StackExchange - "Find angular velocity given pair of rotations")
    KDL::Rotation A = Pose_1.M * Pose_2.M.Inverse();
    KDL::Vector ang_vel = A.GetRot() / dt.toSec();      // GetRot gives the rot. axis whose norm is the angle of rot.

    // Return the twist
    return KDL::Twist(lin_vel, ang_vel);
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::TwistController, controller_interface::ControllerBase)
