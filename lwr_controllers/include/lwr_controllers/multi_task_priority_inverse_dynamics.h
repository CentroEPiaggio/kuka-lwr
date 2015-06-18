#ifndef LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_DYNAMICS_H
#define LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_DYNAMICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/MultiPriorityTask.h>

#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/scoped_ptr.hpp>
#include <sstream>

namespace lwr_controllers
{
	class MultiTaskPriorityInverseDynamics: public controller_interface::PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		MultiTaskPriorityInverseDynamics();
		~MultiTaskPriorityInverseDynamics();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const lwr_controllers::MultiPriorityTask::ConstPtr &msg);
		void set_marker(KDL::Frame x, int index, int id);

	private:
		ros::Subscriber sub_command_;
		ros::Publisher pub_error_;
		ros::Publisher pub_marker_;

		std_msgs::Float64MultiArray msg_err_;
		visualization_msgs::MarkerArray msg_marker_;
		std::stringstream sstr_;

		KDL::JntArray qdot_last_;

		KDL::Frame x_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity
		KDL::Twist x_dot_dot_;	//current e-e acceleration
		std::vector<KDL::Frame> x_des_;	//desired pose

		KDL::Twist x_err_;	// position error

		KDL::JntArray Kp_,Kd_;	// velocity error, position error

		KDL::JntArray tau_;	// control torque

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		std::vector<KDL::Jacobian> J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))

		Eigen::MatrixXd J_pinv_;	// Jacobian pseudo-inv

		Eigen::Matrix<double,6,1> e_ref_;	// reference error
		Eigen::Matrix<double,7,7> I_;		// Identity
		Eigen::Matrix<double,7,7> N_trans_;	// null-space matrix
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;

		int first_step_;	// first step flag
		int msg_id_;		// marker message id
		int cmd_flag_;		// command received flag
		int ntasks_;		// # of task
		std::vector<bool> on_target_flag_;	
		std::vector<int> links_index_;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif