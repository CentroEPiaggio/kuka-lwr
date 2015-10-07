#include <pluginlib/class_list_macros.h>
#include <math.h>

#include <lwr_controllers/gravity_compensation.h>

namespace lwr_controllers 
{
    GravityCompensation::GravityCompensation() {}
    GravityCompensation::~GravityCompensation() {}
    
    bool GravityCompensation::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
        // find stiffness (dummy) joints; this is necessary until proper position/stiffness/damping interface exists
        // for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        // {
        //     joint_stiffness_handles_.push_back(robot->getHandle(it->getJoint().getName()+"_stiffness"));
        // }
        
        // ROS_DEBUG("found %lu stiffness handles", joint_stiffness_handles_.size());
        
        // previous_stiffness_.resize(joint_stiffness_handles_.size());
        
        return true;
    }
    
    void GravityCompensation::starting(const ros::Time& time)
    {
        // for(size_t i=0; i<joint_handles_.size(); i++)
        // {
        //    previous_stiffness_[i] = joint_stiffness_handles_[i].getPosition();
        // }
    }
    
    void GravityCompensation::update(const ros::Time& time, const ros::Duration& period)
    {
        // update the commanded position to the actual, so that the robot doesn't 
        // go back at full speed to the last commanded position when the stiffness 
        // is raised again
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
            //joint_handles_[i].setCommand(joint_handles_[i].getPosition());
            joint_handles_[i].setCommand( 0.0 );
            // joint_stiffness_handles_[i].setCommand(DEFAULT_STIFFNESS);
        }
    }
    
    void GravityCompensation::stopping(const ros::Time& time)
    {
        //for(size_t i=0; i<joint_handles_.size(); i++)
        //{
        //    joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
        //}
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::GravityCompensation, controller_interface::ControllerBase)
