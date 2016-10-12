#ifndef LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_H

// ROS added
#include <geometry_msgs/Wrench.h>
#include <lwr_controllers/Stiffness.h>

// KDL added
#include <kdl/stiffness.hpp>

// BOOST added
#include <boost/scoped_ptr.hpp>

// Base class with useful URDF parsing and kdl chain generator
#include "lwr_controllers/KinematicChainControllerBase.h"

// The format of the command specification
#include "lwr_controllers/CartesianImpedanceReference.h"

namespace lwr_controllers
{
	class CartesianImpedanceController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		CartesianImpedanceController();
		~CartesianImpedanceController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void stopping(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const lwr_controllers::CartesianImpedanceReference::ConstPtr &msg);
		void updateFT(const geometry_msgs::Wrench::ConstPtr &msg);

	private:
		
		// Just to subscribe to the command topic
		ros::Subscriber sub_command_;
		ros::Subscriber sub_ft_measures_;

		// Cartesian vars
		KDL::Frame x_cur_;
		KDL::Frame x_des_;
		KDL::Twist x_err_;

		// The jacobian at q_msg
		KDL::Jacobian J_;

		// Measured and desired force and torque, they need to be expressed in the same ref as x, and J
		KDL::Wrench f_cur_;
		KDL::Wrench f_des_;
		KDL::Wrench f_error_;

		// That is, desired kx, ky, kz, krx, kry, krz, they need to be expressed in the same ref as x, and J
		KDL::Stiffness k_des_;

		// This class does not exist, should we ask for it? 0, for now.
		// KDL::Damping d_des_; 

		// Solver to compute x_cur and x_des
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

		// Solver to compute the Jacobian at q_msr
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

		// Computed torque
		KDL::JntArray tau_cmd_;

		// Because of the lack of the Jacobian transpose in KDL
		void multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest);
	};

}

#endif