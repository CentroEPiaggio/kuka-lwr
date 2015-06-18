#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <lwr_controllers/gravity_compensation.h>

namespace lwr_controllers 
{
    GravityCompensation::GravityCompensation() {}
    GravityCompensation::~GravityCompensation() {}
    
    bool GravityCompensation::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n);
        
        // find stiffness (dummy) joints; this is necessary until proper position/stiffness/damping interface exists
        for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        {
            joint_stiffness_handles_.push_back(robot->getHandle(it->getJoint().getName()+"_stiffness"));
        }
        
        ROS_DEBUG("found %lu stiffness handles", joint_stiffness_handles_.size());
        
        sub_command_ = nh_.subscribe("command", 1, &GravityCompensation::command, this);
        sub_stiffness_ = nh_.subscribe("set_stiffness", 1, &GravityCompensation::set_stiffness, this);
        
        previous_stiffness_.resize(joint_stiffness_handles_.size());
        stiffness_ = DEFAULT_STIFFNESS;
        enabled_ = true;
        
        return true;		
    }
    
    void GravityCompensation::starting(const ros::Time& time)
    {
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
            previous_stiffness_[i] = joint_stiffness_handles_[i].getPosition();
        }
    }
    
    void GravityCompensation::update(const ros::Time& time, const ros::Duration& period)
    {
        if(enabled_)
        {
            // update the commanded position to the actual, so that the robot doesn't 
            // go back at full speed to the last commanded position when the stiffness 
            // is raised again
            for(size_t i=0; i<joint_handles_.size(); i++) 
            {
                joint_handles_[i].setCommand(joint_handles_[i].getPosition());
                joint_stiffness_handles_[i].setCommand(stiffness_);
            }
        } else {
            for(size_t i=0; i<joint_handles_.size(); i++) 
            {
                joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
            }
        }        
    }
    
    void GravityCompensation::command(const std_msgs::Bool::ConstPtr &msg)
    {
        // TODO: if enabling, save last stiffness?
        enabled_ = msg->data;
    }
    
    void GravityCompensation::set_stiffness(const std_msgs::Float32::ConstPtr &msg)
    {
        stiffness_ = msg->data;
    }
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::GravityCompensation, controller_interface::ControllerBase)
