#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <lwr_controllers/dynamic_sliding_mode_controller_task_space.h>

namespace lwr_controllers 
{
	DynamicSlidingModeControllerTaskSpace::DynamicSlidingModeControllerTaskSpace() {}
	DynamicSlidingModeControllerTaskSpace::~DynamicSlidingModeControllerTaskSpace() {}

	bool DynamicSlidingModeControllerTaskSpace::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		nh_ = n;

		// get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("DynamicSlidingModeControllerTaskSpace: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
		    return false;
		}

		if (!nh_.getParam("root_name", root_name))
		{
		    ROS_ERROR_STREAM("DynamicSlidingModeControllerTaskSpace: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
		    return false;
		}

		if (!nh_.getParam("tip_name", tip_name))
		{
		    ROS_ERROR_STREAM("DynamicSlidingModeControllerTaskSpace: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
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

		sub_command_ = nh_.subscribe("command_configuration", 1, &DynamicSlidingModeControllerTaskSpace::command_configuration, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &DynamicSlidingModeControllerTaskSpace::set_gains, this);

		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
		pub_traj_ = nh_.advertise<std_msgs::Float64MultiArray>("traj", 1000);
		pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

		return true;
	}

	void DynamicSlidingModeControllerTaskSpace::starting(const ros::Time& time)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
  			sigma_(i) = 0;

  			// coefficients > 0
  			Kd_(i) = 10;
  			gamma_(i) = 1;
  			alpha_(i) = 12;
  			lambda_(i) = 1;
  			k_(i) = 3;
    	}


    	Kp = 200;
    	Ki = 1; 
    	Kd = 5;

    	for (int i = 0; i < PIDs_.size(); i++)
    		PIDs_[i].initPid(Kp,Ki,Kd,0.1,-0.1);
    	//ROS_INFO("PIDs gains are: Kp = %f, Ki = %f, Kd = %f",Kp,Ki,Kd);

    	//x_des_ = KDL::Frame(KDL::Rotation::RPY(0,0,0),KDL::Vector(-0.4,0.3,1.5));

    	cmd_flag_ = 0;
    	step_ = 0;
	}

	void DynamicSlidingModeControllerTaskSpace::update(const ros::Time& time, const ros::Duration& period)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    	} 

    	if (cmd_flag_) 
    	{
	    	// computing forward kinematics
		    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_);

		    // computing Jacobian J(q)
		    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

		    // computing Jacobian pseudo-inversion
		    pseudo_inverse(J_.data,J_pinv_);

		    // computing end-effector position/orientation error w.r.t. desired frame
		    x_err_ = diff(x_,x_des_);
		    
		    /* Trying quaternions, it seems to work better 

		    // end-effector position/orientation error
	    	x_err_.vel = (x_des_.p - x_.p);

	    	// getting quaternion from rotation matrix
	    	x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
	    	x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

	    	skew_symmetric(quat_des_.v,skew_);

	    	for (int i = 0; i < skew_.rows(); i++)
	    	{
	    		v_temp_(i) = 0.0;
	    		for (int k = 0; k < skew_.cols(); k++)
	    			v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
	    	}

	    	x_err_.rot = (quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v) - v_temp_; 
			*/
	    	// clearing error msg before publishing
    		msg_err_.data.clear();

		    for(int i = 0; i < e_ref_.size(); i++)
			{
				e_ref_(i) = x_err_(i);
		    	msg_err_.data.push_back(e_ref_(i));
			}

			joint_des_states_.qdot.data = J_pinv_*e_ref_;

			joint_des_states_.q.data = joint_msr_states_.q.data + period.toSec()*joint_des_states_.qdot.data;

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

	    	Sq_.data = S_.data + Sd_.data;//- Sd_.data;

	    	// computing sigma_dot as sgn(Sq)
	    	for (int i = 0; i < joint_handles_.size(); i++)
	    		sigma_dot_(i) = -(Sq_(i) < 0) + (Sq_(i) > 0); 

	    	// integrating sigma_dot
	    	sigma_.data += period.toSec()*sigma_dot_.data;

	    	//for (int i = 0; i < joint_handles_.size(); i++)
	    		//sigma_(i) += period.toSec()*pow(Sq_(i),0.5);

	    	// computing Sr
	    	Sr_.data = Sq_.data + gamma_.data.cwiseProduct(sigma_.data);

	    	// computing tau
	    	tau_.data = -Kd_.data.cwiseProduct(Sr_.data);

	    	step_++;

	    	if (Equal(x_,x_des_,0.05))
	    	{
	    		
	    		ROS_INFO("On target");
	    		cmd_flag_ = 0;
	    		return;
	    	}
	    } 

    	// set controls for joints
    	for (int i = 0; i < joint_handles_.size(); i++)
    	{	
    		if (!cmd_flag_)
    			tau_(i) = PIDs_[i].computeCommand(joint_des_states_.q(i) - joint_msr_states_.q(i),period);//Kd_(i)*(alpha_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i)) + (joint_des_states_.qdot(i) - joint_msr_states_.qdot(i))) ;

	    	joint_handles_[i].setCommand(tau_(i));
    	}

    	// publishing markers for visualization in rviz
    	pub_marker_.publish(msg_marker_);
    	msg_id_++;

	    // publishing error for all tasks as an array of ntasks*6
	    pub_error_.publish(msg_err_);
	    // publishing actual and desired trajectory for each task (links) as an array of ntasks*3
	    pub_pose_.publish(msg_pose_);
	    pub_traj_.publish(msg_traj_);
	    ros::spinOnce();

	}

	void DynamicSlidingModeControllerTaskSpace::command_configuration(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if (msg->data.size() == 6)
		{
			x_des_ = KDL::Frame(KDL::Rotation::RPY(msg->data[3],
												   msg->data[4],
												   msg->data[5]),
								KDL::Vector(msg->data[0],
											msg->data[1],
											msg->data[2]));

			step_ = 0;
			cmd_flag_ = 1;
			//SetToZero(joint_des_states_);
		}
		else
		{
			ROS_INFO("Tasks parameters are [x,y,z,roll,pitch,yaw]");
			return;
		}	
	}

	void DynamicSlidingModeControllerTaskSpace::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
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

	void DynamicSlidingModeControllerTaskSpace::set_marker(KDL::Frame x, int id)
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

PLUGINLIB_EXPORT_CLASS(lwr_controllers::DynamicSlidingModeControllerTaskSpace, controller_interface::ControllerBase)
