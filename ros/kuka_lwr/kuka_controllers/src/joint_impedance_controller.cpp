
#include "effort_controllers/joint_impedance_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>

#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <pluginlib/class_list_macros.h>


namespace kuka_controllers {

	JointImpedanceController::JointImpedanceController()
	{}

	JointImpedanceController::~JointImpedanceController()
	{

	}

	bool JointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{

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

// Construct an URDF model from the xml string


		std::string xml_string;

		if (n.hasParam(robot_description))
		{
			n.getParam(robot_description.c_str(), xml_string);
		}

		else
		{
			ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
			n.shutdown();
			return false;
		}

		if (xml_string.size() == 0)
		{
			ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
			n.shutdown();
			return false;
		}

		ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
/// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
			ROS_ERROR("Failed to parse urdf file");
			n.shutdown();
			return false;
		}
		ROS_INFO("Successfully parsed urdf file");

		KDL::Tree kdl_tree_;
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_)){
			ROS_ERROR("Failed to construct kdl tree");
			n.shutdown();
			return false;
		}


// Populate the KDL chain
		if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
		{
			ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
			ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
			ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
			ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
			ROS_ERROR_STREAM("  The segments are:");

			KDL::SegmentMap segment_map = kdl_tree_.getSegments();
			KDL::SegmentMap::iterator it;

			for( it=segment_map.begin();
				it != segment_map.end();
				it++ )
			{
				ROS_ERROR_STREAM( "    "<<(*it).first);
			}

			return false;
		}


		ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
		ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());



// Get joint handles for all of the joints in the chain
		for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin()+1;
			it != kdl_chain_.segments.end(); ++it)
		{
			joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
			ROS_DEBUG("%s", it->getJoint().getName().c_str() );
		}


		K_.resize(kdl_chain_.getNrOfJoints());
		D_.resize(kdl_chain_.getNrOfJoints());

		ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

		for (int i = 0; i < joint_handles_.size(); ++i){
			if ( !nh_.getParam("stiffness_gains", K_(i) ) ){
				ROS_WARN("Stiffness gain not set in yalm file, Using %f", K_(i));
			}
		}
		for (int i = 0; i < joint_handles_.size(); ++i){
			if ( !nh_.getParam("damping_gains", D_(i)) ){
				ROS_WARN("Damping gain not set in yalm file, Using %f", D_(i));
			}
		}



		id_solver_gravity_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		dotq_msr_.resize(kdl_chain_.getNrOfJoints());
		q_msr_.resize(kdl_chain_.getNrOfJoints());
		q_des_.resize(kdl_chain_.getNrOfJoints());
		tau_des_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		tau_gravity_.resize(kdl_chain_.getNrOfJoints());
// K_.resize(kdl_chain_.getNrOfJoints());
// D_.resize(kdl_chain_.getNrOfJoints());

		sub_gains_ = nh_.subscribe("gains", 1, &JointImpedanceController::setGains, this);
		sub_posture_ = nh_.subscribe("command_configuration", 1, &JointImpedanceController::commandConfiguration, this);


		return true;


	}

	void JointImpedanceController::starting(const ros::Time& time)
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

	void JointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
	{

// get joint positions	
		for(size_t i=0; i<joint_handles_.size(); i++) {
			dotq_msr_.q(i) = joint_handles_[i].getPosition();
			q_msr_(i) = dotq_msr_.q(i);
			dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
		}

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


} // namespace

PLUGINLIB_EXPORT_CLASS( kuka_controllers::JointImpedanceController, controller_interface::ControllerBase)
