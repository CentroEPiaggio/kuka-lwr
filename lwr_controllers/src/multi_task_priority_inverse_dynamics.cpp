#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>

#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <lwr_controllers/multi_task_priority_inverse_dynamics.h>

namespace lwr_controllers 
{
	MultiTaskPriorityInverseDynamics::MultiTaskPriorityInverseDynamics() {}
	MultiTaskPriorityInverseDynamics::~MultiTaskPriorityInverseDynamics() {}

	bool MultiTaskPriorityInverseDynamics::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

		qdot_last_.resize(kdl_chain_.getNrOfJoints());
		tau_.resize(kdl_chain_.getNrOfJoints());
		J_.resize(kdl_chain_.getNrOfJoints());
		J_dot_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kd_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe("command", 1, &MultiTaskPriorityInverseDynamics::command, this);

		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("marker",1000);

		return true;
	}

	void MultiTaskPriorityInverseDynamics::starting(const ros::Time& time)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    		joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    		Kp_(i) = 100;
  			Kd_(i) = 20;
    	}

    	I_ = Eigen::Matrix<double,7,7>::Identity(7,7);

    	first_step_ = 0;
    	cmd_flag_ = 0;

	}

	void MultiTaskPriorityInverseDynamics::update(const ros::Time& time, const ros::Duration& period)
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
			// computing Inertia, Coriolis and Gravity matrices
	    	id_solver_->JntToMass(joint_msr_states_.q, M_);
	    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
	    	id_solver_->JntToGravity(joint_msr_states_.q, G_);
	    	// deleting gravity contribute
	    	G_.data.setZero();
	    	
	    	// computing the inverse of M_ now, since it will be used often
	    	pseudo_inverse(M_.data,M_inv_,false);

    		// resetting N and tau(t=0) for the highest priority task
    		N_trans_ = I_;	
    		SetToZero(tau_);

    		for (int index = 0; index < ntasks_; index++)
    		{
		    	// computing Jacobian J(q)
		    	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_,links_index_[index]);

		    	if (first_step_)
		    	{
		    		J_last_[index] = J_;
		    		first_step_ = 0;
		    		return;
		    	}

		    	// computing the derivative of Jacobian J_dot(q) through numerical differentiation
		    	J_dot_.data = (J_.data - J_last_[index].data)/period.toSec();

		    	// computing forward kinematics
		    	fk_pos_solver_->JntToCart(joint_msr_states_.q,x_,links_index_[index]);

		    	// setting marker parameters
		    	set_marker(x_,index,msg_id_);

		    	// computing end-effector position/orientation error w.r.t. desired frame
		    	x_err_ = diff(x_,x_des_[index]);

		    	x_dot_ = J_.data*joint_msr_states_.qdot.data;    	

		    	// setting error reference
		    	for(int i = 0; i < e_ref_.size(); i++)
 		    	{
		    		// e = -Kd*x_dot + Kp*(x_des - x)
		    		e_ref_(i) =  (-Kd_(i)*(x_dot_(i)) + Kp_(i)*x_err_(i));
	    			msg_err_.data.push_back(e_ref_(i));
		    	}

		    	// computing b = J*M^-1*(c+g) - J_dot*q_dot
		    	b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

		    	// computing omega = J*M^-1*N^T*J
		    	omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();

		    	// computing lambda = omega^-1
		    	pseudo_inverse(omega_,lambda_);

		    	// finally, computing the torque tau
		    	tau_.data = tau_.data + J_.data.transpose()*lambda_*(e_ref_ + b_ - J_.data*M_inv_*tau_.data);

		    	// stop condition
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
 
		    	// updating N_^T
		    	N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;

		    	// saving J_ of the last step
		    	J_last_[index] = J_;
		    }		
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
	    ros::spinOnce();

	}

	void MultiTaskPriorityInverseDynamics::command(const lwr_controllers::MultiPriorityTask::ConstPtr &msg)
	{
		if (msg->links.size() == msg->tasks.size()/6)
		{
			ntasks_ = msg->links.size();
			ROS_INFO("Number of tasks: %d",ntasks_);
			// Dynamically resize desired postures and links index when a message arrives
			x_des_.resize(ntasks_);
			links_index_.resize(ntasks_);
			on_target_flag_.resize(ntasks_);
			msg_marker_.markers.resize(ntasks_);
			J_last_.resize(ntasks_);
			for (int i = 0; i < ntasks_; i++)
				J_last_[i].resize(kdl_chain_.getNrOfJoints());
			msg_id_ = 0;

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

			first_step_ = 1;
			cmd_flag_ = 1;
		}	
		else
		{
			ROS_INFO("The number of links index and tasks must be the same");
			ROS_INFO("Tasks parameters are [x,y,x,roll,pitch,yaw]");
			return;
		}
		
	}

	void MultiTaskPriorityInverseDynamics::set_marker(KDL::Frame x, int index, int id)
	{			
				sstr_.str("");
				sstr_.clear();

				if (links_index_[index] == -1)
					sstr_<<"end_effector";		
				else
					sstr_<<"link_"<<(links_index_[index]-1);


				msg_marker_.markers[index].header.frame_id = "world";
				msg_marker_.markers[index].header.stamp = ros::Time();
				msg_marker_.markers[index].ns = sstr_.str();
				msg_marker_.markers[index].id = id;
				msg_marker_.markers[index].type = visualization_msgs::Marker::SPHERE;
				msg_marker_.markers[index].action = visualization_msgs::Marker::ADD;
				msg_marker_.markers[index].pose.position.x = x.p(0);
				msg_marker_.markers[index].pose.position.y = x.p(1);
				msg_marker_.markers[index].pose.position.z = x.p(2);
				msg_marker_.markers[index].pose.orientation.x = 0.0;
				msg_marker_.markers[index].pose.orientation.y = 0.0;
				msg_marker_.markers[index].pose.orientation.z = 0.0;
				msg_marker_.markers[index].pose.orientation.w = 1.0;
				msg_marker_.markers[index].scale.x = 0.01;
				msg_marker_.markers[index].scale.y = 0.01;
				msg_marker_.markers[index].scale.z = 0.01;
				msg_marker_.markers[index].color.a = 1.0;
				msg_marker_.markers[index].color.r = 0.0;
				msg_marker_.markers[index].color.g = 1.0;
				msg_marker_.markers[index].color.b = 0.0;	
	}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::MultiTaskPriorityInverseDynamics, controller_interface::ControllerBase)
