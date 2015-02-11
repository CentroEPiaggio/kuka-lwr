#ifndef LWR_CONTROLLERS__BACKSTEPPING_CONTROL_H
#define LWR_CONTROLLERS__BACKSTEPPING_CONTROL_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <control_toolbox/pid.h>
#include <vector>

#include <lwr_controllers/MultiPriorityTask.h>

namespace lwr_controllers
{
	class BacksteppingController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:
		BacksteppingController();
		~BacksteppingController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command_configuration(const lwr_controllers::MultiPriorityTask::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_marker(KDL::Frame x, int id);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_command_;
		ros::Subscriber sub_gains_;
		ros::Publisher pub_error_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_traj_;
		ros::Publisher pub_marker_;

		std_msgs::Float64MultiArray msg_err_;
		std_msgs::Float64MultiArray msg_pose_;
		std_msgs::Float64MultiArray msg_traj_;
		visualization_msgs::Marker msg_marker_;

		KDL::Chain kdl_chain_;

		KDL::JntArrayAcc joint_msr_states_, joint_des_states_;	// joint states (measured and desired)
		KDL::JntArrayAcc joint_ref_;

		KDL::Frame x_,x0_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		Eigen::Matrix<double,6,1> x_des_dot_;
		Eigen::Matrix<double,6,1> x_des_dotdot_;

		Eigen::Matrix<double,6,1> x_ref_dot_;
		Eigen::Matrix<double,6,1> x_ref_dotdot_;

		struct limits_
		{
			KDL::JntArray min;
			KDL::JntArray max;
			KDL::JntArray center;
		} joint_limits_;

		KDL::Twist x_err_;

		KDL::JntArray Kp_,Kd_;

		KDL::JntArray tau_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))
		Eigen::MatrixXd J_pinv_;

		int step_;
		int first_step_;
		int msg_id_;
		int cmd_flag_;

		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

		std::vector<hardware_interface::JointHandle> joint_handles_;
		std::vector<control_toolbox::Pid> PIDs_;
		double Kp,Ki,Kd;
	};

}

#endif