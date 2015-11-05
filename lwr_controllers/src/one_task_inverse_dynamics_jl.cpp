#include <lwr_controllers/one_task_inverse_dynamics_jl.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>

#include <cmath>


namespace lwr_controllers 
{
	OneTaskInverseDynamicsJL::OneTaskInverseDynamicsJL() {}
	OneTaskInverseDynamicsJL::~OneTaskInverseDynamicsJL() {}

	bool OneTaskInverseDynamicsJL::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        
		qdot_last_.resize(kdl_chain_.getNrOfJoints());
		tau_.resize(kdl_chain_.getNrOfJoints());
		J_.resize(kdl_chain_.getNrOfJoints());
		J_dot_.resize(kdl_chain_.getNrOfJoints());
		J_star_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kd_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		J_last_.resize(kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe("command", 1, &OneTaskInverseDynamicsJL::command, this);
		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		// pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
		// pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);


		return true;
	}

	void OneTaskInverseDynamicsJL::starting(const ros::Time& time)
	{
		// get joint positions
  		for(unsigned int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    		joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    		Kp_(i) = 50;
  			Kd_(i) = 10;
    	}

    	I_ = Eigen::Matrix<double,7,7>::Identity(7,7);
    	e_ref_ = Eigen::Matrix<double,6,1>::Zero();
      fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);

    	first_step_ = 1;
    	cmd_flag_ = 0;
    	step_ = 0;

	}

	void OneTaskInverseDynamicsJL::update(const ros::Time& time, const ros::Duration& period)
	{
		// get joint positions


  		for(unsigned int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    	}

    	// clearing msgs before publishing
    	msg_err_.data.clear();
    	// msg_pose_.data.clear();
    	
    	// if (cmd_flag_)
    	// {
  		// resetting N and tau(t=0) for the highest priority task
  		N_trans_ = I_;	
  		SetToZero(tau_);

  		// computing Inertia, Coriolis and Gravity matrices
	    id_solver_->JntToMass(joint_msr_states_.q, M_);
	    id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
	    id_solver_->JntToGravity(joint_msr_states_.q, G_);
	    G_.data.setZero();

	    // computing the inverse of M_ now, since it will be used often
	    pseudo_inverse(M_.data,M_inv_,false); //M_inv_ = M_.data.inverse(); 


    	// computing Jacobian J(q)
    	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

    	// computing the distance from the mid points of the joint ranges as objective function to be minimized
    	phi_ = task_objective_function(joint_msr_states_.q);

      fk_pos_solver_->JntToCart(joint_msr_states_.q,x_);

    	// using the first step to compute jacobian of the tasks
    	if (first_step_ == 1)
    	{
    		J_last_ = J_;
    		phi_last_ = phi_;
    		first_step_ = 0;
    		return;
    	}

      // std::cout << "timer: " << timer_control_.toSec() << std::endl;
    	// computing the derivative of Jacobian J_dot(q) through numerical differentiation
    	J_dot_.data = (J_.data - J_last_.data)/period.toSec();

    	// computing forward kinematics
    	// set_marker(x_,msg_id_);

    	// computing end-effector position/orientation error w.r.t. desired frame

      x_err_ = diff(x_,x_des_);
      

    	x_dot_ = J_.data*joint_msr_states_.qdot.data;    	

    	// setting error reference
    	for(unsigned int i = 0; i < e_ref_.size(); i++)
	    {
    		// e = x_des_dotdot + Kd*(x_des_dot - x_dot) + Kp*(x_des - x)
    		e_ref_(i) =  -Kd_(i)*(x_dot_(i)) + Kp_(i)*x_err_(i);
  			msg_err_.data.push_back(x_err_(i));
    	}

  		// computing b = J*M^-1*(c+g) - J_dot*q_dot
  		b_ = J_.data*M_inv_*(C_.data + G_.data) - J_dot_.data*joint_msr_states_.qdot.data;

    	// computing omega = J*M^-1*N^T*J
    	omega_ = J_.data*M_inv_*N_trans_*J_.data.transpose();

    	// computing lambda = omega^-1
    	pseudo_inverse(omega_,lambda_);
    	//lambda_ = omega_.inverse();

    	// computing nullspace
    	N_trans_ = N_trans_ - J_.data.transpose()*lambda_*J_.data*M_inv_;  	    		

    	// finally, computing the torque tau
    	tau_.data = J_.data.transpose()*lambda_*(e_ref_ + b_);// + N_trans_*(Eigen::Matrix<double,7,1>::Identity(7,1)*(phi_ - phi_last_)/(period.toSec()));

    	// saving J_ and phi of the last iteration
    	J_last_ = J_;
    	phi_last_ = phi_;
	

    	// set controls for joints
    	for (unsigned int i = 0; i < joint_handles_.size(); i++)
    	{
    			joint_handles_[i].setCommand(tau_(i));
    	}


	    // publishing error 
	    pub_error_.publish(msg_err_);
	    ros::spinOnce();

	}

	void OneTaskInverseDynamicsJL::command(const lwr_controllers::PoseRPY::ConstPtr &msg)
	{	
		KDL::Frame frame_des_;

		switch(msg->id)
		{
			case 0:
			frame_des_ = KDL::Frame(
					KDL::Rotation::RPY(msg->orientation.roll,
						 			  msg->orientation.pitch,
								 	  msg->orientation.yaw),
					KDL::Vector(msg->position.x,
								msg->position.y,
								msg->position.z));
			break;
	
			case 1: // position only
			frame_des_ = KDL::Frame(
				KDL::Vector(msg->position.x,
							msg->position.y,
							msg->position.z));
			break;
		
			case 2: // orientation only
			frame_des_ = KDL::Frame(
				KDL::Rotation::RPY(msg->orientation.roll,
				   	 			   msg->orientation.pitch,
								   msg->orientation.yaw));
			break;

			default:
			ROS_INFO("Wrong message ID");
			return;
		}
		x_des_ = frame_des_;

    first_step_ = 1;
	}


	double OneTaskInverseDynamicsJL::task_objective_function(KDL::JntArray q)
	{
		double sum = 0;
		double temp;
		int N = q.data.size();

		for (int i = 0; i < N; i++)
		{
			temp = ((q(i) - joint_limits_.center(i))/(joint_limits_.max(i) - joint_limits_.min(i)));
			sum += temp*temp;
		}

		sum /= 2*N;

		return -sum;
	}
}





PLUGINLIB_EXPORT_CLASS(lwr_controllers::OneTaskInverseDynamicsJL, controller_interface::ControllerBase)

