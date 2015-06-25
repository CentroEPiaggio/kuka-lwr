#ifndef ARM_STATE_CONTROLLER_H
#define ARM_STATE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <boost/scoped_ptr.hpp>

namespace arm_state_controller
{
    class ArmStateController: public controller_interface::KinematicChainControllerBase<hardware_interface::JointStateInterface>
    {
    public:
        
        ArmStateController();
        ~ArmStateController();
        
        bool init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
        
    private:
                
        boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
        
        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        boost::scoped_ptr<KDL::Vector> gravity_;
        boost::scoped_ptr<KDL::JntArray> joint_position_;
        boost::scoped_ptr<KDL::JntArray> joint_velocity_;
        boost::scoped_ptr<KDL::JntArray> joint_acceleration_;
        boost::scoped_ptr<KDL::Wrenches> joint_wrenches_;
        boost::scoped_ptr<KDL::JntArray> joint_effort_est_;
        
        ros::Time last_publish_time_;
        double publish_rate_;
        
    };
}

#endif