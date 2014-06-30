
#include "effort_controllers/joint_impedance_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace kuka_controllers {

JointImpedanceController::JointImpedanceController()
{}

JointImpedanceController::~JointImpedanceController()
{

}

bool JointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name)) 
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Load PID Controller using gains set on parameter server
  if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  // sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointImpedanceController::setCommandCB, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void JointImpedanceController::starting(const ros::Time& time)
{
  double pos_command = joint_.getPosition();

}

void JointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{

  command_struct_.position_ = 0.0;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);

}


} // namespace

PLUGINLIB_EXPORT_CLASS( kuka_controllers::JointImpedanceController, controller_interface::ControllerBase)

















/*
#include <algorithm>

#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <pluginlib/class_list_macros.h>

#include "effort_controllers/joint_impedance_controller.h"

namespace kuka_controllers {
	JointImpedanceController::JointImpedanceController(){

	}

	JointImpedanceController::~JointImpedanceController(){
		sub_gains_.shutdown();
		sub_posture_.shutdown();

	}

	bool JointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n){

		nh_ = n;

    // get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description)){
			ROS_ERROR_STREAM("JointImpedanceController: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
			return false;
		}
		if (!nh_.getParam("root_name", root_name)){
			ROS_ERROR_STREAM("JointImpedanceController: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
			return false;
		}
		if (!nh_.getParam("tip_name", tip_name)){
			ROS_ERROR_STREAM("JointImpedanceController: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
			return false;
		}

	// Get the gravity vector (direction and magnitude)
		KDL::Vector gravity_ = KDL::Vector::Zero();
		gravity_(2) = 9.81;
		// // gravity_[2] = 9.81;
		// XmlRpc::XmlRpcValue xml_gravity;
		// nh.getParam("gravity", xml_gravity);
		// if(xml_gravity.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		// 	ROS_WARN_STREAM("Gravity list parameter not found under: "
		// 		<<nh.getNamespace()<<"/gravity, assuming no gravity.");
		// } else {
		// 	for (int i = 0; i < 3; ++i) {
		// 		ROS_ASSERT(xml_gravity[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		// 		gravity_[i] = static_cast<double>(xml_gravity[i]);
		// 	}
		// }

    // Construct an URDF model from the xml string
		urdf::Model urdf_model;
		urdf_model.initString(robot_description);

    // Get a KDL tree from the robot URDF
		KDL::Tree kdl_tree;
		if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
			ROS_ERROR("Failed to construct kdl tree from URDF model");
			return false;
		}

    // Populate the KDL chain
		if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
		{
			ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
			ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
			ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
			ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
			ROS_ERROR_STREAM("  The segments are:");

			KDL::SegmentMap segment_map = kdl_tree.getSegments();
			KDL::SegmentMap::iterator it;

			for( it=segment_map.begin();
				it != segment_map.end();
				it++ )
			{
				ROS_ERROR_STREAM( "    "<<(*it).first);
			}

			return false;
		}

    // Get joint handles for all of the joints in the chain
		for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin();
			it != kdl_chain_.segments.end();
			++it)
		{
			joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
		}


		for (int i = 0; i < joint_handles_.size(); ++i){
   			if ( !nh_.getParam("stiffness_gains", K_(i)) ){
   				ROS_WARN("Stiffness gain not set in yalm file, Using %f", K_(i));
   			}
		}
		for (int i = 0; i < joint_handles_.size(); ++i){
   			if ( !nh_.getParam("damping_gains", D_(i)) ){
   				ROS_WARN("Damping gain not set in yalm file, Using %f", D_(i));
   			}
		}
   		

	// Create inverse dynamics solver
		// id_solver_.reset( new KDL::ChainIdSolver_RNE( kdl_chain_, gravity_) );
		id_solver_gravity_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		dotq_msr_.resize(kdl_chain_.getNrOfJoints());
		q_msr_.resize(kdl_chain_.getNrOfJoints());
		q_des_.resize(kdl_chain_.getNrOfJoints());
		tau_des_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		tau_gravity_.resize(kdl_chain_.getNrOfJoints());
		K_.resize(kdl_chain_.getNrOfJoints());
		D_.resize(kdl_chain_.getNrOfJoints());

		sub_gains_ = n.subscribe("gains", 2*kdl_chain_.getNrOfJoints(), &JointImpedanceController::setGains, this);
  		sub_posture_ = n.subscribe("command_configuration", kdl_chain_.getNrOfJoints(), &JointImpedanceController::commandConfiguration, this);

		return true;

	}

	void JointImpedanceController::starting(const ros::Time &time)
	{

    // get joint positions
		for(size_t i=0; i<joint_handles_.size(); i++) {
			K_(i) = 300.0;
			D_(i) = 0.7;
			tau_des_(i) = 0.0;
			dotq_msr_.q(i) = joint_handles_[i].getPosition();
			q_des_(i) = dotq_msr_.q(i);
			dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
		}

	}

	void JointImpedanceController::update(const ros::Time& time, const ros::Duration& dt)
	{
    // get joint positions	
		for(size_t i=0; i<joint_handles_.size(); i++) {
			dotq_msr_.q(i) = joint_handles_[i].getPosition();
			q_msr_(i) = dotq_msr_.q(i);
			dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
		}
			// dotq_msr_.qdotdot(i) = 0.0;
	//Compute control law
		id_solver_gravity_->JntToGravity( q_msr_ , tau_gravity_ );
		for(size_t i=0; i<joint_handles_.size(); i++) {
			tau_cmd_(i) = K_(i) * (q_des_(i) - q_msr_(i)) + D_(i)*dotq_msr_.qdot(i) + tau_des_(i) + tau_gravity_(i);
			joint_handles_[i].setCommand(tau_cmd_	(i));
		}	

	}

	void JointImpedanceController::commandConfiguration(const std_msgs::Float64MultiArray::ConstPtr &msg){
		if (msg->data.size() == 0) {
			ROS_INFO("Desired configuration must be: %lu imension dimension", joint_handles_.size());
		}
		else if ((int)msg->data.size() != joint_handles_.size()) {
			ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
			return;
		}
		else
		{
			for (unsigned int j = 0; j < joint_handles_.size(); ++j)
				q_des_(j) = msg->data[j];
		}
		
	}

	void JointImpedanceController::setGains(const std_msgs::Float64MultiArray::ConstPtr &msg){

		if (msg->data.size() == 2*joint_handles_.size()){
			for (unsigned int i = 0; i < joint_handles_.size(); ++i)
				K_(i) = msg->data[i];
			for (unsigned int i = joint_handles_.size(); i < 2*joint_handles_.size(); ++i)
				D_(i)= msg->data[i];
		}
		ROS_INFO("New gains K: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
			K_(0), K_(1), K_(2), K_(3), K_(4), K_(5), K_(6));
		ROS_INFO("New gains D: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
			D_(0), D_(1), D_(2), D_(3), D_(4), D_(5), D_(6));

	}


// PLUGINLIB_DECLARE_CLASS(kuka_controllers, JointImpedanceController, kuka_controllers::JointImpedanceController,controller_interface::ControllerBase)

}

PLUGINLIB_EXPORT_CLASS( kuka_controllers::JointImpedanceController, controller_interface::ControllerBase)

*/