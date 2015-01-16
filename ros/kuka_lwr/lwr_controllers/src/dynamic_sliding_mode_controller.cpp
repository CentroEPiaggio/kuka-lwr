#include <lwr_controllers/dynamic_sliding_mode_controller.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>

namespace lwr_controllers 
{
	DynamicSlidingModeController::DynamicSlidingModeController() {}
	DynamicSlidingModeController::~DynamicSlidingModeController() {}

	bool DynamicSlidingModeController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		nh_ = n;

		// get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("DynamicSlidingModeController: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
		    return false;
		}

		if (!nh_.getParam("root_name", root_name))
		{
		    ROS_ERROR_STREAM("DynamicSlidingModeController: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
		    return false;
		}

		if (!nh_.getParam("tip_name", tip_name))
		{
		    ROS_ERROR_STREAM("DynamicSlidingModeController: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
		    return false;
		}
	 
		// Get the gravity vector (direction and magnitude)
		KDL::Vector gravity_ = KDL::Vector::Zero();
		gravity_(2) = -9.81;

		// Construct an URDF model from the xml string
		std::string xml_string;

		if (n.hasParam(robot_description))
			n.getParam(robot_description.c_str(), xml_string);
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
		
		// Get urdf model out of robot_description
		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    ROS_ERROR("Failed to parse urdf file");
		    n.shutdown();
		    return false;
		}
		ROS_INFO("Successfully parsed urdf file");
		
		KDL::Tree kdl_tree_;
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
		{
		    ROS_ERROR("Failed to construct kdl tree");
		    n.shutdown();
		    return false;
		}

		// Parsing joint limits from urdf model
		boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    	boost::shared_ptr<const urdf::Joint> joint_;
    	joint_limits_.min.resize(kdl_tree_.getNrOfJoints());
		joint_limits_.max.resize(kdl_tree_.getNrOfJoints());
		joint_limits_.center.resize(kdl_tree_.getNrOfJoints());
		int index;

    	for (int i = 0; i < kdl_tree_.getNrOfJoints() && link_; i++)
    	{
    		joint_ = model.getJoint(link_->parent_joint->name);  
    		index = kdl_tree_.getNrOfJoints() - i - 1;

    		joint_limits_.min(index) = joint_->limits->lower;
    		joint_limits_.max(index) = joint_->limits->upper;
    		joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

    		link_ = model.getLink(link_->getParent()->name);
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

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      ROS_ERROR_STREAM( "    "<<(*it).first);

		  	return false;
		}

		ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
		ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());

		// Get joint handles for all of the joints in the chain
		for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin()+1; it != kdl_chain_.segments.end(); ++it)
		{
		    joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
		    ROS_DEBUG("%s", it->getJoint().getName().c_str() );
		}

		ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

		joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
		joint_des_states_.resize(kdl_chain_.getNrOfJoints());
		joint_ref_.resize(kdl_chain_.getNrOfJoints());
		tau_.resize(kdl_chain_.getNrOfJoints());
		J_.resize(kdl_chain_.getNrOfJoints());
		PIDs_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		S_.resize(kdl_chain_.getNrOfJoints());
		S0_.resize(kdl_chain_.getNrOfJoints());
		Sd_.resize(kdl_chain_.getNrOfJoints());
		Sq_.resize(kdl_chain_.getNrOfJoints());
		Sr_.resize(kdl_chain_.getNrOfJoints());
		sigma_.resize(kdl_chain_.getNrOfJoints());
		sigma_dot_.resize(kdl_chain_.getNrOfJoints());

		Kd_.resize(kdl_chain_.getNrOfJoints());
		gamma_.resize(kdl_chain_.getNrOfJoints());
		alpha_.resize(kdl_chain_.getNrOfJoints());
		lambda_.resize(kdl_chain_.getNrOfJoints());
		k_.resize(kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe("command_configuration", 1, &DynamicSlidingModeController::command_configuration, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &DynamicSlidingModeController::set_gains, this);

		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
		pub_traj_ = nh_.advertise<std_msgs::Float64MultiArray>("traj", 1000);
		pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

		return true;
	}

	void DynamicSlidingModeController::starting(const ros::Time& time)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
  			sigma_(i) = 0;

  			// coefficients > 0
  			Kd_(i) = 25;
  			gamma_(i) = 1.5;
  			alpha_(i) = 15;
  			//lambda_(i) = 5;
  			k_(i) = 1;
    	}

    	SetToZero(joint_des_states_);

    	Kp = 200;
    	Ki = 1; 
    	Kd = 5;

    	for (int i = 0; i < PIDs_.size(); i++)
    		PIDs_[i].initPid(Kp,Ki,Kd,0.1,-0.1);
    	//ROS_INFO("PIDs gains are: Kp = %f, Ki = %f, Kd = %f",Kp,Ki,Kd);

    	cmd_flag_ = 0;
    	step_ = 0;
	}

	void DynamicSlidingModeController::update(const ros::Time& time, const ros::Duration& period)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    	} 

    	double freq_ = 1.0;

    	joint_des_states_.q(3) = sin(freq_/2*step_*period.toSec());
    	joint_des_states_.qdot(3) = freq_/2*cos(freq_/2*step_*period.toSec());
    	joint_des_states_.q(1) = sin(freq_*step_*period.toSec());
    	joint_des_states_.qdot(1) = freq_*cos(freq_*step_*period.toSec());

    	msg_pose_.data.clear();
    	msg_traj_.data.clear();

    	// updating messages for plot visualization
    	for (int i = 0; i < joint_handles_.size(); i++)
    	{
    		msg_pose_.data.push_back(joint_msr_states_.q(i));
    		msg_traj_.data.push_back(joint_des_states_.q(i));
    	}

    	// computing S
    	S_.data = (joint_msr_states_.qdot.data - joint_des_states_.qdot.data) + alpha_.data.cwiseProduct(joint_msr_states_.q.data - joint_des_states_.q.data);

		//for (int i = 0; i < joint_handles_.size(); i++)
			//S_(i) = (joint_msr_states_.qdot(i) - joint_des_states_.qdot(i)) + alpha_(i)*tanh(lambda_(i)*(joint_msr_states_.q(i) - joint_des_states_.q(i)));

    	// saving S0 on the first step
    	if (step_ == 0)
    		S0_ = S_;

    	// computing Sd
    	for (int i = 0; i < joint_handles_.size(); i++)
    		Sd_(i) = S0_(i)*exp(-k_(i)*(step_*period.toSec()));

    	Sq_.data = S_.data - Sd_.data;

    	// computing sigma_dot as sgn(Sq)
    	for (int i = 0; i < joint_handles_.size(); i++)
    		sigma_dot_(i) = -(Sq_(i) < 0) + (Sq_(i) > 0); 

    	// integrating sigma_dot
    	sigma_.data += period.toSec()*sigma_dot_.data;

    	// computing Sr
    	Sr_.data = Sq_.data + gamma_.data.cwiseProduct(sigma_.data);

    	// computing tau
    	tau_.data = -Kd_.data.cwiseProduct(Sr_.data);

    	step_++; 

    	// set controls for joints
    	for (int i = 0; i < joint_handles_.size(); i++)
    	{	
	    	joint_handles_[i].setCommand(tau_(i));
    	}

    	// publishing markers for visualization in rviz
    	pub_marker_.publish(msg_marker_);
    	msg_id_++;

	    // publishing error for all tasks as an array of ntasks*6
	    pub_error_.publish(msg_err_);
	    // publishing actual and desired trajectory for each joint specified
	    pub_pose_.publish(msg_pose_);
	    pub_traj_.publish(msg_traj_);
	    ros::spinOnce();

	}

	void DynamicSlidingModeController::command_configuration(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{

	}

	void DynamicSlidingModeController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if(msg->data.size() == 3)
		{
			for(int i = 0; i < PIDs_.size(); i++)
				PIDs_[i].setGains(msg->data[0],msg->data[1],msg->data[2],0.3,-0.3);
			ROS_INFO("New gains set: Kp = %f, Ki = %f, Kd = %f",msg->data[0],msg->data[1],msg->data[2]);
		}
		else
			ROS_INFO("PIDs gains needed are 3 (Kp, Ki and Kd)");
	}

	void DynamicSlidingModeController::set_marker(KDL::Frame x, int id)
	{			
				msg_marker_.header.frame_id = "world";
				msg_marker_.header.stamp = ros::Time();
				msg_marker_.ns = "end_effector";
				msg_marker_.id = id;
				msg_marker_.type = visualization_msgs::Marker::SPHERE;
				msg_marker_.action = visualization_msgs::Marker::ADD;
				msg_marker_.pose.position.x = x.p(0);
				msg_marker_.pose.position.y = x.p(1);
				msg_marker_.pose.position.z = x.p(2);
				msg_marker_.pose.orientation.x = 0.0;
				msg_marker_.pose.orientation.y = 0.0;
				msg_marker_.pose.orientation.z = 0.0;
				msg_marker_.pose.orientation.w = 1.0;
				msg_marker_.scale.x = 0.005;
				msg_marker_.scale.y = 0.005;
				msg_marker_.scale.z = 0.005;
				msg_marker_.color.a = 1.0;
				msg_marker_.color.r = 0.0;
				msg_marker_.color.g = 1.0;
				msg_marker_.color.b = 0.0;	
	}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::DynamicSlidingModeController, controller_interface::ControllerBase)
