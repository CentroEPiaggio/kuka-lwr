#include <minimum_effort_inverse_dynamics.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>
#include <misc/pseudo_inversion.h>

namespace kuka_controllers 
{
	MinimumEffortInverseDynamics::MinimumEffortInverseDynamics() {}
	MinimumEffortInverseDynamics::~MinimumEffortInverseDynamics() {}

	bool MinimumEffortInverseDynamics::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		nh_ = n;

		// get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("MinimumEffortInverseDynamics: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
		    return false;
		}

		if (!nh_.getParam("root_name", root_name))
		{
		    ROS_ERROR_STREAM("MinimumEffortInverseDynamics: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
		    return false;
		}

		if (!nh_.getParam("tip_name", tip_name))
		{
		    ROS_ERROR_STREAM("MinimumEffortInverseDynamics: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
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
		qdot_last_.resize(kdl_chain_.getNrOfJoints());
		tau_.resize(kdl_chain_.getNrOfJoints());
		J_.resize(kdl_chain_.getNrOfJoints());
		J_dot_.resize(kdl_chain_.getNrOfJoints());
		J_last_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kd_.resize(kdl_chain_.getNrOfJoints());
		PIDs_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe("command_configuration", 1, &MinimumEffortInverseDynamics::command_configuration, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &MinimumEffortInverseDynamics::set_gains, this);

		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
		pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

		return true;
	}

	void MinimumEffortInverseDynamics::starting(const ros::Time& time)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    		joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    		Kp_(i) = 500;
  			Kd_(i) = 2;
    	}

    	Kp = 200;
    	Ki = 1; 
    	Kd = 5;

    	for (int i = 0; i < PIDs_.size(); i++)
    		PIDs_[i].initPid(Kp,Ki,Kd,0.1,-0.1);
    	//ROS_INFO("PIDs gains are: Kp = %f, Ki = %f, Kd = %f",Kp,Ki,Kd);

    	I_ = Eigen::Matrix<double,7,7>::Identity(7,7);
    	e_ref_ = Eigen::Matrix<double,6,1>::Zero();

    	first_step_ = 1;
    	cmd_flag_ = 1;
    	step_ = 0;

	}

	void MinimumEffortInverseDynamics::update(const ros::Time& time, const ros::Duration& period)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    		joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    	}

    	/* TASK 0 (Main)*/

    	double freq_ = 3.0;
    	double amp_ = 0.1;

		//Desired posture (circle figure)
		x_des_.p(0) = 0.3 + amp_*(1 - cos(freq_*period.toSec()*step_));//0.05*sin(freq_*period.toSec()*step_);
    	x_des_.p(1) = 0.3 + amp_*sin(freq_*period.toSec()*step_);//0.05*sin(2*freq_*period.toSec()*step_);
    	x_des_.p(2) = 1.1;
    	x_des_.M = KDL::Rotation(KDL::Rotation::RPY(0,3.1415,0));

    	//velocity
    	x_des_dot_(0) = amp_*freq_*sin(freq_*period.toSec()*step_);//0.05*freq_*cos(freq_*period.toSec()*step_);
    	x_des_dot_(1) = amp_*freq_*cos(freq_*period.toSec()*step_);//0.05*2*freq_*cos(2*freq_*period.toSec()*step_);
    	x_des_dot_(2) = 0;
    	x_des_dot_(3) = 0;
    	x_des_dot_(4) = 0;
    	x_des_dot_(5) = 0;

    	//acc
    	x_des_dotdot_(0) = amp_*freq_*freq_*cos(freq_*period.toSec()*step_);//-0.05*freq_*freq_*sin(freq_*period.toSec()*step_);
    	x_des_dotdot_(1) = -amp_*freq_*freq_*sin(freq_*period.toSec()*step_);//-0.05*2*2*freq_*freq_*sin(freq_*period.toSec()*step_);
    	x_des_dotdot_(2) = 0;
    	x_des_dotdot_(3) = 0;
    	x_des_dotdot_(4) = 0;
    	x_des_dotdot_(5) = 0;

    	step_++;

    	// clearing msgs before publishing
    	msg_err_.data.clear();
    	msg_pose_.data.clear();
    	
    	if (cmd_flag_)
    	{
    		// resetting N and tau(t=0) for the highest priority task
    		N_trans_ = I_;	
    		SetToZero(tau_);

    		// computing Inertia, Coriolis and Gravity matrices
		    id_solver_->JntToMass(joint_msr_states_.q, M_);
		    id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
		    id_solver_->JntToGravity(joint_msr_states_.q, G_);

		    // computing the inverse of M_ now, since it will be used often
		    pseudo_inverse(M_.data,M_inv_,false); //M_inv_ = M_.data.inverse(); 

	    	// computing Jacobian J(q)
	    	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

	    	// using the first step to compute jacobian of the tasks
	    	if (first_step_)
	    	{
	    		J_last_ = J_;
	    		first_step_ = 0;
	    		return;
	    	}

	    	// computing the derivative of Jacobian J_dot(q) through numerical differentiation
	    	J_dot_.data = (J_.data - J_last_.data)/period.toSec();

	    	// computing forward kinematics
	    	fk_pos_solver_->JntToCart(joint_msr_states_.q,x_);

	    	// pushing x to the pose msg
	    	for (int i = 0; i < 3; i++)
	    		msg_pose_.data.push_back(x_.p(i));

	    	// setting marker parameters
	    	set_marker(x_,msg_id_);

	    	// computing end-effector position/orientation error w.r.t. desired frame
	    	x_err_ = diff(x_,x_des_);
	    	
	    	x_dot_ = J_.data*joint_msr_states_.qdot.data;    	

	    	// setting error reference
	    	for(int i = 0; i < e_ref_.size(); i++)
		    {
	    		// e = x_des_dotdot + Kd*(x_des_dot - x_dot) + Kp*(x_des - x)
	    		e_ref_(i) =  x_des_dotdot_(i) + Kd_(i)*(x_des_dot_(i) - x_dot_(i)) + Kp_(i)*x_err_(i);
    			msg_err_.data.push_back(e_ref_(i));
	    	}

    		// computing b = J*M^-1*(c+g) - J_dot*q_dot
    		b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

	    	// computing omega = J*M^-1*N^T*J
	    	omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();

	    	// computing lambda = omega^-1
	    	pseudo_inverse(omega_,lambda_);

	    	// computing nullspace
	    	N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;

	    	// finally, computing the torque tau
	    	tau_.data = J_.data.transpose()*lambda_*(e_ref_ + b_) + N_trans_*G_.data;

	    	// saving J_ of the last iteration
	    	J_last_ = J_;		
    	}

    	// set controls for joints
    	for (int i = 0; i < joint_handles_.size(); i++)
    	{
    		if(cmd_flag_)
    			joint_handles_[i].setCommand(tau_(i));
    		else
    			joint_handles_[i].setCommand(PIDs_[i].computeCommand(joint_des_states_.q(i) - joint_msr_states_.q(i),period));
    	}

    	// publishing markers for visualization in rviz
    	pub_marker_.publish(msg_marker_);
    	msg_id_++;

	    // publishing error for all tasks as an array of ntasks*6
	    pub_error_.publish(msg_err_);
	    // publishing pose for each task (links) as an array of ntasks*2
	    pub_pose_.publish(msg_pose_);
	    ros::spinOnce();

	}

	void MinimumEffortInverseDynamics::command_configuration(const kuka_controllers::MultiPriorityTask::ConstPtr &msg)
	{
		first_step_ = 1;
		cmd_flag_ = 1;	
	}

	void MinimumEffortInverseDynamics::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
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

	void MinimumEffortInverseDynamics::set_marker(KDL::Frame x, int id)
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

PLUGINLIB_EXPORT_CLASS(kuka_controllers::MinimumEffortInverseDynamics, controller_interface::ControllerBase)
