#ifndef LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor

#include <vector>

namespace lwr_controllers
{
	class ComputedTorqueController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:

		ComputedTorqueController();
		~ComputedTorqueController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command_configuration(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private:

		ros::NodeHandle nh_;
		ros::Subscriber sub_posture_;
		ros::Subscriber sub_gains_;

		KDL::Chain kdl_chain_;
		KDL::JntArrayAcc joint_msr_states_, joint_des_states_;	// joint states (measured and desired)
		KDL::JntArray cmd_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture

		KDL::JntArray tau_cmd_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		KDL::JntArray Kp_, Kv_;	//Position and Velocity gains

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		std::vector<hardware_interface::JointHandle> joint_handles_;

	};
}

#endif