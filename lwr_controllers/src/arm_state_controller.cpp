#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <lwr_controllers/arm_state_controller.h>

namespace arm_state_controller  
{
    ArmStateController::ArmStateController() {}
    ArmStateController::~ArmStateController() {}
    
    bool ArmStateController::init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, n);
        
        // TODO: parse urdf for link mass, inertia, etc.
        
        // TODO: new realtime publisher
        
        gravity_.reset(new KDL::Vector(0.0, 0.0, -9.81)); // TODO: compute from actual robot position (TF?)
        id_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_, gravity_));
        joint_position_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints());
        joint_velocity_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints());
        joint_acceleration_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints());
        joint_wrenches_.reset(new KDL::Wrenches);
        joint_effort_est_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints());
        
        for (unsigned i = 0; i < joint_handles_.size(); i++){
            joint_wrenches_->push_back(KDL::Wrench);
        }
        
        return true;		
    }
    
    void ArmStateController::starting(const ros::Time& time)
    {
        last_publish_time_ = time;
        for (unsigned i = 0; i < joint_handles_.size(); i++){
            joint_position_[i] = joint_handles_[i].getPosition();
            joint_velocity_[i] = joint_handles_[i].getVelocity();
            joint_acceleration_[i] = 0; 
        }
    }
    
    void ArmStateController::update(const ros::Time& time, const ros::Duration& period)
    {
        // TODO: compute gravity force on robot from joint state and urdf data
        
        // TODO: copied from JointStateController, adjust
        // limit rate of publishing
        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){
            
            // try to publish
            if (realtime_pub_->trylock()){
                // we're actually publishing, so increment time
                last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
                
                for (unsigned i = 0; i < joint_handles_.size(); i++){
                    joint_position_[i] = joint_handles_[i].getPosition();
                    joint_velocity_[i] = joint_handles_[i].getVelocity();
                    joint_acceleration_[i] = 0;  // TODO: compute from previous velocity?
                }
                
                // Compute Dynamics 
                int ret = id_solver_->CartToJnt(*joint_position_, 
                                                *joint_velocity_, 
                                                *joint_acceleration_, 
                                                *joint_wrenches_,
                                                *joint_effort_est_);
                if (ret < 0) { 
                    ROS_ERROR("KDL: inverse dynamics ERROR");
                    realtime_pub_->unlock();
                    return;
                }
                
                // populate joint state message
                realtime_pub_->msg_.header.stamp = time;
                for (unsigned i=0; i<joint_handles_.size(); i++){
                    realtime_pub_->msg_.position[i] = joint_handles_[i].getPosition();
                    realtime_pub_->msg_.velocity[i] = joint_handles_[i].getVelocity();
                    realtime_pub_->msg_.effort[i] = joint_handles_[i].getEffort();
                }
                realtime_pub_->unlockAndPublish();
            }
        }   
    }
    
    void ArmStateController::stopping(const ros::Time& time)
    {
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
            joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
        }
    }
    
}

PLUGINLIB_EXPORT_CLASS(arm_state_controller::ArmStateController, controller_interface::ControllerBase)
