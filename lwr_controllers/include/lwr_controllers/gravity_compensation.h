#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace lwr_controllers
{
    class GravityCompensation: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
    {
    public:
        
        GravityCompensation();
        ~GravityCompensation();
        
        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
        
    // private:
        
        // std::vector<float> previous_stiffness_; /// stiffness before activating controller
        
        // hack required as long as there is separate position handle for stiffness
        // std::vector<hardware_interface::JointHandle> joint_stiffness_handles_;
        
        // const static float DEFAULT_STIFFNESS = 0.01;
        
    };
}

#endif