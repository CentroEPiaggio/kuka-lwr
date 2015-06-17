#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <lwr_controllers/computed_torque_controller.h>

namespace lwr_controllers 
{
	ComputedTorqueController::ComputedTorqueController() {}
	ComputedTorqueController::~ComputedTorqueController() {}

	bool ComputedTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
		id_solver_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());

		sub_posture_ = nh_.subscribe("command", 1, &ComputedTorqueController::command, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &ComputedTorqueController::set_gains, this);

		return true;		
	}

	void ComputedTorqueController::starting(const ros::Time& time)
	{
  		// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{

  			Kp_(i) = 300.0;
  			Kv_(i) = 0.7;
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    	}

    	lambda = 0.1;	// lower values: flatter
    	cmd_flag_ = 0;	
    	step_ = 0;

    	ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );

    }

    void ComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
    {
    	// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    	}

    	
    	if(cmd_flag_)
    	{
    		// reaching desired joint position using a hyperbolic tangent function
    		for(size_t i=0; i<joint_handles_.size(); i++)
    		{
    			joint_des_states_.q(i) = cmd_states_(i)*1/2*(1+ tanh(lambda*step_-M_PI)); 
    			joint_des_states_.qdot(i) = cmd_states_(i)*1/2*lambda*(1/(cosh(M_PI-lambda*step_)*cosh(M_PI-lambda*step_))); // 1/(cosh^2) = sech^2
    			joint_des_states_.qdotdot(i) = cmd_states_(i)*lambda*lambda*(1/(cosh(M_PI-step_)*cosh(M_PI-step_)))*tanh(M_PI-step_);
    		}
    		++step_;

    		if(joint_des_states_.q == cmd_states_)
    		{
    			cmd_flag_ = 0;	//reset command flag
    			step_ = 0;
    			ROS_INFO("Posture OK");
    		}
    	}

    	// computing Inertia, Coriolis and Gravity matrices
    	id_solver_->JntToMass(joint_msr_states_.q, M_);
    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    	id_solver_->JntToGravity(joint_msr_states_.q, G_);

		for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			// control law
			tau_cmd_(i) = M_(i,i)*(joint_des_states_.qdotdot(i) + Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i))) + C_(i)*joint_msr_states_.qdot(i) + G_(i);
		   	joint_handles_[i].setCommand(tau_cmd_(i));
		}	

    }

    void ComputedTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
    	if(msg->data.size() == 0)
    		ROS_INFO("Desired configuration must be of dimension %lu", joint_handles_.size());
    	else if(msg->data.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    		return;
    	}
    	else
    	{
    		for (unsigned int i = 0; i<joint_handles_.size(); i++)
    			cmd_states_(i) = msg->data[i];

    		cmd_flag_ = 1;
    	}

	}

	void ComputedTorqueController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if(msg->data.size() == 2*joint_handles_.size())
		{
			for(unsigned int i = 0; i < joint_handles_.size(); i++)
			{
				Kp_(i) = msg->data[i];
				Kv_(i) = msg->data[i + joint_handles_.size()];
			}
		}
		else
			ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

		ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

		ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		ROS_INFO("New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));

	}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::ComputedTorqueController, controller_interface::ControllerBase)
