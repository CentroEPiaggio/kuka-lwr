#ifndef KUKA_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H
#define KUKA_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <control_toolbox/pid.h>
#include <kuka_controllers/MultiPriorityTask.h>
#include <vector>

namespace kuka_controllers
{
	class MultiTaskPriorityInverseKinematics: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:
		MultiTaskPriorityInverseKinematics();
		~MultiTaskPriorityInverseKinematics();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command_configuration(const kuka_controllers::MultiPriorityTask::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_marker(KDL::Frame x, int index, int id);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_command_;
		ros::Subscriber sub_gains_;
		ros::Publisher pub_error_;
		ros::Publisher pub_marker_;

		std_msgs::Float64MultiArray msg_err_;
		visualization_msgs::MarkerArray msg_marker_;
		std::stringstream sstr_;

		KDL::Chain kdl_chain_;

		KDL::JntArrayAcc joint_msr_states_, joint_des_states_;	// joint states (measured and desired)

		KDL::Frame x_;		//current pose
		std::vector<KDL::Frame> x_des_;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray tau_cmd_;

		KDL::Jacobian J_;	//Jacobian
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::MatrixXd J_pinv_;//<double,7,6> J_pinv_;

		Eigen::Matrix<double,6,1> e_dot_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> P_;

		int msg_id_;
		int cmd_flag_;
		int ntasks_;
		std::vector<bool> on_target_flag_;
		std::vector<int> links_index_;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;

		std::vector<hardware_interface::JointHandle> joint_handles_;
		std::vector<control_toolbox::Pid> PIDs_;
		double Kp,Ki,Kd;
	};

}

#endif