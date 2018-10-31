#ifndef LWR_CONTROLLERS__TWIST_CONTROLLER_H
#define LWR_CONTROLLERS__TWIST_CONTROLLER_H

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include "KinematicChainControllerBase.h"
#include "kdl/chaindynparam.hpp"


namespace lwr_controllers {

  class TwistController : public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface> {

  public:
  // Default Constructor
  TwistController();

  // Default Destructor
  ~TwistController();

  // Controller functions
  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
	void starting(const ros::Time& time);
	void update(const ros::Time& time, const ros::Duration& period);
	void command(const geometry_msgs::Twist::ConstPtr &msg);

  private:
  // Flag for position holding if twist_des_ = 0 (IN CASE IT IS NEEDED)
  int cmd_flag_;

  // ROS Variables
  ros::Subscriber sub_command_;           // For listening to the command topic

  // Twists and frames of different time instants
  KDL::Twist twist_des_;                  // The current desired twist (for this iteration) written by subscriber
  KDL::Twist twist_des_new_;              // The desired twist (for this iteration) inclusive of previous it. twist error
  KDL::Twist twist_ach_;                  // The twist achieved (in previous iteration)
  KDL::Twist twist_des_old_;              // The past desired twist (in previous iteration)
  KDL::Twist twist_error_;                // Error between past desired and past achieved
  KDL::Twist twist_real_;                 // The real twist computed using real joint states
  KDL::Twist twist_error_real_;           // Error between past desired and past achieved real
  KDL::Twist twist_check_;                // Error for checking if the pseudo inv gave q dots that achieve desired twist
  KDL::Frame P_ee_;                       // Current end-effecter cartesian pose (achieved after previous iteration)
  KDL::Frame P_ee_old_;                   // The end-effector pose at the beginning of past iteration
  KDL::Frame P_ee_real_;                  // The end-effector pose computed from the real joint states
  KDL::Frame P_ee_real_old_;              // The real end-effector pose achieved in prev. iteration

  // KDL joint related variables
  KDL::JntArray joint_states_;            // Joint state containing q and q_dot (updated with integration)
  KDL::JntArray joint_states_real_;       // Joint state containing q and q_dot (updated with getPosition)
  KDL::JntArray joint_states_old_;        // Joint state containing q and q_dot (old joints_states_ from previous iteration)
  KDL::JntArray joint_vel_comm_;          // Joint velocity required for twist following (computed with J_pinv)
  KDL::JntArray joint_comm_;              // Joint command given to position interface
  KDL::Jacobian jacobian_;                // Robot jacobian

  // Eigen Variables
  Eigen::MatrixXd jacobian_pinv_;         // Pseudo inverse of the jacobian

  // Solvers (twist, jacobian and gravity solvers)
  // boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;              // Not needed???
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pos_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // Auxiliary functions
  KDL::Twist computeTwistFromFrames(KDL::Frame Pose_1, KDL::Frame Pose_2, ros::Duration dt);

  };

}

#endif //LWR_CONTROLLERS__TWIST_CONTROLLER_H
