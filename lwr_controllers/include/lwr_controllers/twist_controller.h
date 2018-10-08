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

    // Measured and commanded twists
    KDL::Twist twist_meas_, twist_des_;
    KDL::FrameVel tmp_twist_meas_;    // Used for forward twist computing
    KDL::Twist error_;                // Error between desired and measured
    KDL::Twist twist_tmp_;            // Used temp. for comparing with old twists

  private:
    // Flag for position holding if twist_des_ = 0
    int cmd_flag_;

    // ROS Variables
    ros::Subscriber sub_command_;     // For listening to the command topic

    // KDL Variables
    KDL::JntArrayVel joint_states_;   // Joint state containing q and q_dot
    KDL::JntArray joint_vel_comm_;    // Joint velocity for twist following
    KDL::JntArray joint_comm_;        // Joint command given to position interf.
    KDL::Jacobian jacobian_;          // Robot jacobian

    // Eigen Variables
    Eigen::MatrixXd jacobian_pinv_;   // Pseudo inverse of the jacobian

    // Solvers (twist, jacobian and gravity solvers)
    boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  };

}



#endif //LWR_CONTROLLERS__TWIST_CONTROLLER_H
