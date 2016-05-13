#ifndef LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/MultiPriorityTask.h>

#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/scoped_ptr.hpp>
#include <sstream>

namespace lwr_controllers
{
	class MultiTaskPriorityInverseKinematics: public controller_interface::PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		MultiTaskPriorityInverseKinematics();
		~MultiTaskPriorityInverseKinematics();

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
	};

}

#endif