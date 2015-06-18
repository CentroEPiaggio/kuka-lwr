#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>

#include <utils/pseudo_inversion.h>
#include <lwr_controllers/backstepping_controller.h>

namespace lwr_controllers 
{
	BacksteppingController::BacksteppingController() {}
	BacksteppingController::~BacksteppingController() {}

	bool BacksteppingController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
		KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

		joint_ref_.resize(kdl_chain_.getNrOfJoints());
		tau_.resize(kdl_chain_.getNrOfJoints());
		J_.resize(kdl_chain_.getNrOfJoints());
		J_dot_.resize(kdl_chain_.getNrOfJoints());
		J_last_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kd_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		sub_command_ = nh_.subscribe("command", 1, &BacksteppingController::command, this);

		pub_error_ = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
		pub_pose_ = nh_.advertise<std_msgs::Float64MultiArray>("pose", 1000);
		pub_traj_ = nh_.advertise<std_msgs::Float64MultiArray>("traj", 1000);
		pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker",1000);

		return true;
	}

	void BacksteppingController::starting(const ros::Time& time)
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

    	fk_pos_solver_->JntToCart(joint_msr_states_.q,x0_);

    	ROS_INFO("x: %f, y: %f, z:%f",x0_.p(0),x0_.p(1),x0_.p(2));

    	first_step_ = 1;
    	cmd_flag_ = 1;
    	step_ = 0;

	}

	void BacksteppingController::update(const ros::Time& time, const ros::Duration& period)
	{
		// get joint positions
  		for(int i=0; i < joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		//joint_des_states_.q(i) = joint_msr_states_.q(i);
    		//joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    	}

    	/* TASK 0 (Main)*/

    	double freq_ = 3.0;
    	double amp_ = 0.1;

		//Desired posture 8 figure (circle figure)
		x_des_.p(0) = /*0.3*/0.25 + /*amp_*(1 - cos(freq_*period.toSec()*step_));*/amp_/2*sin(freq_*period.toSec()*step_);
    	x_des_.p(1) = /*0.3*/0 + /*amp_*sin(freq_*period.toSec()*step_);*/amp_/2*sin(2*freq_*period.toSec()*step_);
    	x_des_.p(2) = /*1.1*/1.75;
    	x_des_.M = KDL::Rotation(KDL::Rotation::RPY(0,0,0));//RPY(0,3.1415,0));

    	//velocity
    	x_des_dot_(0) = /*amp_*freq_*sin(freq_*period.toSec()*step_);*/amp_/2*freq_*cos(freq_*period.toSec()*step_);
    	x_des_dot_(1) = /*amp_*freq_*cos(freq_*period.toSec()*step_);*/amp_/2*2*freq_*cos(2*freq_*period.toSec()*step_);
    	x_des_dot_(2) = 0;
    	x_des_dot_(3) = 0;
    	x_des_dot_(4) = 0;
    	x_des_dot_(5) = 0;

    	//acc
    	x_des_dotdot_(0) = /*amp_*freq_*freq_*cos(freq_*period.toSec()*step_);*/-amp_/2*freq_*freq_*sin(freq_*period.toSec()*step_);
    	x_des_dotdot_(1) = /*-amp_*freq_*freq_*sin(freq_*period.toSec()*step_);*/-amp_/2*2*2*freq_*freq_*sin(freq_*period.toSec()*step_);
    	x_des_dotdot_(2) = 0;
    	x_des_dotdot_(3) = 0;
    	x_des_dotdot_(4) = 0;
    	x_des_dotdot_(5) = 0;

    	step_++;

    	// clearing msgs before publishing
    	msg_err_.data.clear();
    	msg_pose_.data.clear();
    	msg_traj_.data.clear();
    	
    	if (cmd_flag_)
    	{
    		// computing Inertia, Coriolis and Gravity matrices
		    id_solver_->JntToMass(joint_msr_states_.q, M_);
		    id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
		    id_solver_->JntToGravity(joint_msr_states_.q, G_);

	    	// computing Jacobian J(q)
	    	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

	    	// computing Jacobian pseudo-inversion
	    	pseudo_inverse(J_.data,J_pinv_);

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
	    	{
	    		msg_pose_.data.push_back(x_.p(i));
	    		msg_traj_.data.push_back(x_des_.p(i));
	    	}

	    	// setting marker parameters
	    	set_marker(x_,msg_id_);

	    	// computing end-effector position/orientation error w.r.t. desired frame
	    	x_err_ = diff(x_,x_des_);
	    	
	    	// computing task space velocity (of end-effector)
	    	x_dot_ = J_.data*joint_msr_states_.qdot.data;

	    	// computing reference variables in task space
	    	for (int i = 0; i < 6; i++)
	    	{
	    		x_ref_dot_(i) = x_des_dot_(i) + Kp_(i)*x_err_(i);
	    		x_ref_dotdot_(i) = x_des_dotdot_(i) + Kd_(i)*(x_des_dot_(i) - x_dot_(i)) + Kp_(i)*x_err_(i);
	    	}

	    	// computing reference variable in joint space
	    	joint_ref_.qdot.data = J_pinv_*x_ref_dot_;

	    	joint_ref_.qdotdot.data = J_pinv_*(x_ref_dotdot_ - J_dot_.data*joint_msr_states_.qdot.data);

	    	joint_des_states_.qdot.data = J_pinv_*x_des_dot_;

	    	joint_des_states_.q.data += period.toSec()*joint_des_states_.qdot.data;

	    	// finally, computing the torque tau
	    	tau_.data = M_.data*joint_ref_.qdotdot.data + C_.data.cwiseProduct(joint_ref_.qdot.data) + G_.data + /*K_d*/(joint_ref_.qdot.data - joint_msr_states_.qdot.data) + (joint_des_states_.q.data - joint_msr_states_.q.data); 

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
	    // publishing actual and desired trajectory for each task (links) as an array of ntasks*3
	    pub_pose_.publish(msg_pose_);
	    pub_traj_.publish(msg_traj_);
	    ros::spinOnce();

	}

	void BacksteppingController::command(const lwr_controllers::MultiPriorityTask::ConstPtr &msg)
	{
		step_ = 0;
		first_step_ = 1;
		cmd_flag_ = 1;	
	}

	void BacksteppingController::set_marker(KDL::Frame x, int id)
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

PLUGINLIB_EXPORT_CLASS(lwr_controllers::BacksteppingController, controller_interface::ControllerBase)
