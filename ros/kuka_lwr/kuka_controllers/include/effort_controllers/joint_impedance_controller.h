
#ifndef KUKA_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define KUKA_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor

#include <vector>

/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)
	
*/

namespace kuka_controllers
{

	class JointImpedanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:

		JointImpedanceController();
		~JointImpedanceController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);

		void update(const ros::Time& time, const ros::Duration& period);
		void commandConfiguration(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private:

		ros::NodeHandle nh_;
		ros::Subscriber sub_gains_;
		ros::Subscriber sub_posture_;

		KDL::Chain kdl_chain_;
		KDL::JntArrayVel dotq_msr_;
		KDL::JntArray q_msr_, q_des_;
		KDL::JntArray tau_des_, tau_cmd_, tau_gravity_;
		KDL::JntArray K_, D_;

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_gravity_;

		std::vector<hardware_interface::JointHandle> joint_handles_;

	};

} // namespace

#endif
