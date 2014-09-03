#include "multi_priority_task_inverse_kinematics.h"
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>
#include <misc/pseudo_inversion.h>
#include <misc/skew_symmetric.h>

namespace kuka_controllers 
{
	MultiPriorityTaskInverseKinematics::MultiPriorityTaskInverseKinematics() {}
	MultiPriorityTaskInverseKinematics::~MultiPriorityTaskInverseKinematics() {}

	bool MultiPriorityTaskInverseKinematics::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		nh_ = n;

		// get URDF and name of root and tip from the parameter server
		std::string robot_description, root_name, tip_name;

		if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
		{
		    ROS_ERROR_STREAM("MultiPriorityTaskInverseKinematics: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
		    return false;
		}

		if (!nh_.getParam("root_name", root_name))
		{
		    ROS_ERROR_STREAM("MultiPriorityTaskInverseKinematics: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
		    return false;
		}

		if (!nh_.getParam("tip_name", tip_name))
		{
		    ROS_ERROR_STREAM("MultiPriorityTaskInverseKinematics: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
		    return false;
		}
	 
		// Get the gravity vector (direction and magnitude)
		KDL::Vector gravity_ = KDL::Vector::Zero();
		gravity_(2) = 9.81;

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
		ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
		ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR(kdl_chain_,*fk_pos_solver_,*ik_vel_solver_));

		joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
		joint_des_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		J_.resize(kdl_chain_.getNrOfJoints());
		J_star_.resize(kdl_chain_.getNrOfJoints());
		PIDs_.resize(kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe("command_configuration", 1, &MultiPriorityTaskInverseKinematics::command_configuration, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &MultiPriorityTaskInverseKinematics::set_gains, this);

		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);

		return true;
	}

	void MultiPriorityTaskInverseKinematics::starting(const ros::Time& time)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    	}

    	Kp = 300;
    	Ki = 1;
    	Kd = 3;

    	for (int i = 0; i < PIDs_.size(); i++)
    		PIDs_[i].initPid(Kp,Ki,Kd,0.2,-0.2);
    	ROS_INFO("PIDs gains are: Kp = %f, Ki = %f, Kd = %f",Kp,Ki,Kd);

    	I_ = Eigen::Matrix<double,7,7>::Identity(7,7);
    	e_dot_ = Eigen::Matrix<double,6,1>::Zero();

    	cmd_flag_ = 0;
	}

	void MultiPriorityTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
	{

		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    	}

    	// clearing error msg before publishing
    	msg_err_.data.clear();

    	if (cmd_flag_)
    	{
    		// resetting P and qdot(t=0) for the highest priority task
    		P_ = I_;	
    		SetToZero(joint_des_states_.qdot);

    		for (int index = 0; index < ntasks_; index++)
    		{
		    	// computing Jacobian
		    	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_,links_index_[index]);

		    	// computing forward kinematics
		    	fk_pos_solver_->JntToCart(joint_msr_states_.q,x_,links_index_[index]);

		    	/* <---- old code for position/orientation error based on quaternion units (*might* still be useful) ---->

		    	// end-effector position/orientation error
		    	//x_err_.vel = x_des_[index].p - x_.p;

		    	// getting quaternion from rotation matrix
		    	x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
		    	x_des_[index].M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

		    	skew_symmetric(quat_des_.v,skew_);

		    	for (int i = 0; i < skew_.rows(); i++)
		    	{
		    		v_temp_(i) = 0.0;
		    		for (int k = 0; k < skew_.cols(); k++)
		    			v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
		    	}

		    	//x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_; 

		    	<----- end old code ----> */

		    	// computing end-effector position/orientation error w.r.t. desired frame
		    	x_err_ = diff(x_,x_des_[index]);

		    	for(int i = 0; i < e_dot_.size(); i++)
		    	{
		    		e_dot_(i) = x_err_(i);
	    			msg_err_.data.push_back(e_dot_(i));
		    	}

		    	// computing (J[i]*P[i-1])^pinv
		    	J_star_.data = J_.data*P_;
		    	pseudo_inverse_DLS(J_star_.data,J_pinv_);

		    	// computing q_dot (qdot(i) = qdot[i-1] + (J[i]*P[i-1])^pinv*(x_err[i] - J[i]*qdot[i-1]))
		    	joint_des_states_.qdot.data = joint_des_states_.qdot.data + J_pinv_*(e_dot_ - J_.data*joint_des_states_.qdot.data);

		    	if (!on_target_flag_[index])
		    	{
			    	if (Equal(x_,x_des_[index],0.01))
			    	{
			    		ROS_INFO("Task %d on target",index);
			    		on_target_flag_[index] = true;
			    		if (index == (ntasks_ - 1))
			    			cmd_flag_ = 0;
			    	}
			    }

		    	// updating P_ (it mustn't make use of the damped pseudo inverse)
		    	pseudo_inverse(J_star_.data,J_pinv_);
		    	P_ = P_ - J_pinv_*J_star_.data;
		    }

		    // integrating q_dot -> getting q (Euler method)
		    for (int i = 0; i < joint_handles_.size(); i++)
		    	joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);			
    	}

    	// set controls for joints
    	for (int i = 0; i < joint_handles_.size(); i++)
    	{
    		tau_cmd_(i) = PIDs_[i].computeCommand(joint_des_states_.q(i) - joint_msr_states_.q(i),period);
    		joint_handles_[i].setCommand(tau_cmd_(i));
    	}

	    // publishing error for all tasks as an array of ntasks*6
	    pub_error_.publish(msg_err_);
	    ros::spinOnce();

	}

	void MultiPriorityTaskInverseKinematics::command_configuration(const kuka_controllers::MultiPriorityTask::ConstPtr &msg)
	{
		if (msg->links.size() == msg->tasks.size()/6)
		{
			ntasks_ = msg->links.size();
			ROS_INFO("Number of tasks: %d",ntasks_);
			// Dynamically resize desired postures and links index when a message arrives
			x_des_.resize(ntasks_);
			links_index_.resize(ntasks_);
			on_target_flag_.resize(ntasks_);

			for (int i = 0; i < ntasks_; i++)
			{
					if (msg->links[i] == -1)	// adjust index
						links_index_[i] = msg->links[i];
					else if (msg->links[i] >= 1 && msg->links[i] <=joint_handles_.size())
						links_index_[i] = msg->links[i] + 1;
					else
					{
						ROS_INFO("Links index must be within 1 and %ld. (-1 is end-effector)",joint_handles_.size());
						return;
					}
					
					x_des_[i] = KDL::Frame(
									KDL::Rotation::RPY(msg->tasks[i*6 + 3],
													   msg->tasks[i*6 + 4],
													   msg->tasks[i*6 + 5]),
									KDL::Vector(msg->tasks[i*6],
												msg->tasks[i*6 + 1],
												msg->tasks[i*6 + 2]));

					on_target_flag_[i] = false;
			}

			cmd_flag_ = 1;
		}	
		else
		{
			ROS_INFO("The number of links index and tasks must be the same");
			ROS_INFO("Tasks parameters are [x,y,x,roll,pitch,yaw]");
			return;
		}
		
	}

	void MultiPriorityTaskInverseKinematics::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if(msg->data.size() == 3)
		{
			for(int i = 0; i < PIDs_.size(); i++)
				PIDs_[i].setGains(msg->data[0],msg->data[1],msg->data[2],0.3,-0.3);
			ROS_INFO("New gains set: Kp = %f, Kd = %f",msg->data[0],msg->data[1],msg->data[2]);
		}
		else
			ROS_INFO("PIDs gains needed are 3 (Kp, Ki and Kd)");
	}
}

PLUGINLIB_EXPORT_CLASS(kuka_controllers::MultiPriorityTaskInverseKinematics, controller_interface::ControllerBase)
