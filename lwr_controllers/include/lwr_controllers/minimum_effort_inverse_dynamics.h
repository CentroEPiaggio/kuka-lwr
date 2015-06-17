#ifndef LWR_CONTROLLERS__MINIMUM_EFFORT_INVERSE_DYNAMICS_H
#define LWR_CONTROLLERS__MINIMUM_EFFORT_INVERSE_DYNAMICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/MultiPriorityTask.h>

#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <control_msgs/JointControllerState.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <vector>


namespace lwr_controllers
{
	class MinimumEffortInverseDynamics: public controller_interface::PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		MinimumEffortInverseDynamics();
		~MinimumEffortInverseDynamics();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const lwr_controllers::MultiPriorityTask::ConstPtr &msg);
		void set_marker(KDL::Frame x, int id);

	private:
		ros::Subscriber sub_command_;
		ros::Publisher pub_error_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_marker_;

		std_msgs::Float64MultiArray msg_err_;
		std_msgs::Float64MultiArray msg_pose_;
		visualization_msgs::Marker msg_marker_;
        
		KDL::JntArray qdot_last_;

		KDL::Frame x_,x0_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		KDL::Twist x_des_dot_;
		KDL::Twist x_des_dotdot_;

		KDL::Twist x_err_;

		KDL::JntArray Kp_,Kd_;

		KDL::JntArray tau_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))

		Eigen::Matrix<double,6,1> e_ref_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> N_trans_;
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;

		int step_;
		int first_step_;
		int msg_id_;
		int cmd_flag_;

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif