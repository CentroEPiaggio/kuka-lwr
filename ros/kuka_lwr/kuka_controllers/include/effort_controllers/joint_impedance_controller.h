
#ifndef kuka_controllers__JOINT_INPEDANCE_CONTROLLER_H
#define kuka_controllers__JOINT_INPEDANCE_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
//#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <eigen_conversions/eigen_kdl.h>
// #include <eigen_conversions/eigen_msg.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp> //to be tested
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

namespace kuka_controllers
{

class JointImpedanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double velocity_; // Last commanded velocity
    bool has_velocity_; // false if no velocity command has been specified
  };

  JointImpedanceController();
  ~JointImpedanceController();
 
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

  void starting(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
	void commandConfiguration(const std_msgs::Float64MultiArray::ConstPtr &msg);
	void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg);
	// bool getTreeFromURDF(KDL::Tree &tree);


  hardware_interface::JointHandle joint_;
  boost::shared_ptr<const urdf::Joint> joint_urdf_;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;

  void setCommandCB(const std_msgs::Float64ConstPtr& msg);

  void enforceJointLimits(double &command);

  	enum { Joints = 7 };
	ros::NodeHandle nh_;
	ros::Subscriber sub_gains_;
	ros::Subscriber sub_posture_;

	KDL::Chain kdl_chain_;
	KDL::JntArrayVel dotq_msr_;
	KDL::JntArray q_msr_, q_des_;
	KDL::JntArray tau_des_, tau_cmd_, tau_gravity_;
	KDL::JntArray K_, D_;

	boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
	boost::scoped_ptr<KDL::ChainDynParam> id_solver_gravity_;

	std::vector<hardware_interface::JointHandle> joint_handles_;

};

} // namespace

#endif












/*#ifndef KUKA_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_H
#define KUKA_CONTROLLERS__JOINT_IMPEDANCE_CONTROLLER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
//#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <eigen_conversions/eigen_kdl.h>
// #include <eigen_conversions/eigen_msg.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp> //to be tested
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>


// Control mode 30 in kuka robot

// tau_cmd = K*(q_des - q_msr) + D*dotq_msr + tau_des + f(q,dotq,ddotd)


namespace kuka_controllers
{

	class JointImpedanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	    JointImpedanceController();
	    ~JointImpedanceController();

	    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
	    void starting(const ros::Time& time);
	    void update(const ros::Time& time, const ros::Duration& period);
	    void commandConfiguration(const std_msgs::Float64MultiArray::ConstPtr &msg);
	    void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg);
	   
	private:

	enum { Joints = 7 };
	ros::NodeHandle nh_;
	ros::Subscriber sub_gains_;
	ros::Subscriber sub_posture_;

	KDL::Chain kdl_chain_;
	KDL::JntArrayVel dotq_msr_;
	KDL::JntArray q_msr_, q_des_;
	KDL::JntArray tau_des_, tau_cmd_, tau_gravity_;
	KDL::JntArray K_, D_;

	boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
	boost::scoped_ptr<KDL::ChainDynParam> id_solver_gravity_;

	std::vector<hardware_interface::JointHandle> joint_handles_;

	};

} //namespace

#endif

*/