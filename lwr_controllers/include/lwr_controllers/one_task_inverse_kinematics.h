#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_KINEMATICS_H

#include "KinematicChainControllerBase.h"
#include <lwr_controllers/MultiPriorityTask.h>
#include <lwr_controllers/PoseRPY.h>
#include <lwr_controllers/PIDgains.h>

#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <control_toolbox/pid.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

namespace lwr_controllers
{
	class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		OneTaskInverseKinematics();
		~OneTaskInverseKinematics();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command_configuration(const lwr_controllers::PoseRPY::ConstPtr &msg);

	private:
		ros::Subscriber sub_command_;
		ros::Subscriber sub_gains_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray tau_cmd_;

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

		std::vector<control_toolbox::Pid> PIDs_;
	};

}

#endif
