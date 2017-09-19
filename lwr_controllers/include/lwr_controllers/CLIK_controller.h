// authors: Piero Guardati, Federico Nesti

#ifndef LWR_CONTROLLERS__CLIK_CONTROLLER_H
#define LWR_CONTROLLERS__CLIK_CONTROLLER_H

#include "KinematicChainControllerBase.h"
#include "lwr_controllers/PoseRPY.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <std_msgs/String.h> //PEPIPA
#include <std_msgs/Int16.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
namespace lwr_controllers
{
	class CLIKController: 

	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>

	{
	public:
		CLIKController();			//Vuoto
		~CLIKController(); 		//Vuoto

		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const lwr_controllers::PoseRPY::ConstPtr &msg);

	private:
		void controller_type(const std_msgs::Int16::ConstPtr &msg);
		void extend_chain(ros::NodeHandle &n);
		void command2(const geometry_msgs::Pose::ConstPtr &msg);
		ros::Publisher pub_initial_position;
		ros::Subscriber sub_command_,sub_command2;
		ros::Subscriber sub_control_type;
		ros::Subscriber sub_gains_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;
		Eigen::Matrix<double,3,3> skew_;

		// extended kdl chain
	    KDL::Chain extended_chain_;

	    // p_wrist_ee
	    KDL::Vector p_wrist_ee_;

	    // Weight Matrix
	    Eigen::MatrixXd W_gain;

	    int control_type;
		double alpha1, alpha2, stiffness_max_;
	    KDL::Frame x_des_2;	//desired pose



	tf::TransformBroadcaster tf_desired_hand_pose, elbow_reference;



		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> 			jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> 		fk_pos_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> 			ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> 			ik_pos_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
	};

}

#endif
