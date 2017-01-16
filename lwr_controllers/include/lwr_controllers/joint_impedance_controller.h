
#ifndef LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace lwr_controllers
{

	class JointImpedanceController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:

		JointImpedanceController();
		~JointImpedanceController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);

		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void setParam(const std_msgs::Float64MultiArray::ConstPtr &msg, KDL::JntArray* array, std::string s);
        
	private:

		ros::Subscriber sub_stiffness_, sub_damping_, sub_add_torque_;
		ros::Subscriber sub_posture_;

		KDL::JntArray q_des_;
		KDL::JntArray tau_des_;
		KDL::JntArray K_, D_;

	};

} // namespace

#endif
