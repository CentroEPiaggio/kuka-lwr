#ifndef LWR_CONTROLLERS__DYNAMIC_SLIDING_MODE_CONTROL_TASK_SPACE_H
#define LWR_CONTROLLERS__DYNAMIC_SLIDING_MODE_CONTROL_TASK_SPACE_H

#include "PIDKinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>
 
namespace lwr_controllers
{
	class DynamicSlidingModeControllerTaskSpace: public controller_interface::PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:
		DynamicSlidingModeControllerTaskSpace();
		~DynamicSlidingModeControllerTaskSpace();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_marker(KDL::Frame x, int id);

	private:
		ros::Subscriber sub_command_;
		ros::Publisher pub_error_;
		ros::Publisher pub_pose_;
		ros::Publisher pub_traj_;
		ros::Publisher pub_marker_;

		std_msgs::Float64MultiArray msg_err_;
		std_msgs::Float64MultiArray msg_pose_;
		std_msgs::Float64MultiArray msg_traj_;
		visualization_msgs::Marker msg_marker_;

		KDL::JntArrayAcc joint_ref_;

		KDL::Frame x_,x0_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		Eigen::Matrix<double,6,1> x_des_dot_;
		Eigen::Matrix<double,6,1> x_des_dotdot_;

		Eigen::Matrix<double,6,1> e_ref_;
		Eigen::Matrix<double,6,1> x_ref_dot_;
		Eigen::Matrix<double,6,1> x_ref_dotdot_;

		Eigen::Matrix<double,3,3> skew_;

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;

		KDL::JntArray S_;
		KDL::JntArray S0_;
		KDL::JntArray Sd_;
		KDL::JntArray Sq_;
		KDL::JntArray Sr_;
		KDL::JntArray sigma_;
		KDL::JntArray sigma_dot_;

		// coefficients
		KDL::JntArray Kd_;
		KDL::JntArray gamma_;
		KDL::JntArray alpha_;
		KDL::JntArray lambda_;
		KDL::JntArray k_;

		KDL::Twist x_err_;

		KDL::JntArray tau_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		Eigen::MatrixXd J_pinv_;

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