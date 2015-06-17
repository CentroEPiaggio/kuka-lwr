#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace lwr_controllers
{
    class GravityCompensation: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
    {
    public:
        
        GravityCompensation();
        ~GravityCompensation();
        
        bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void command(const std_msgs::Bool::ConstPtr &msg);
        void set_stiffness(const std_msgs::Float32::ConstPtr &msg);
        
    private:
        
        ros::Subscriber sub_command_;
        ros::Subscriber sub_stiffness_;
        
        bool enabled_;
        float stiffness_;
        std::vector<float> previous_stiffness_; /// stiffness before activating controller
        
        // hack required as long as there is separate position handle for stiffness
        std::vector<hardware_interface::JointHandle> joint_stiffness_handles_;
        
        const static float DEFAULT_STIFFNESS = 0.01;
        
    };
}

#endif