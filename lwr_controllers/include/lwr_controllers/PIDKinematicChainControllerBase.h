#ifndef PID_KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define PID_KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include "KinematicChainControllerBase.h"

#include <control_toolbox/pid.h>

namespace controller_interface
{
    template<typename JI>
	class PIDKinematicChainControllerBase: public KinematicChainControllerBase<JI>
	{
        
	public:
		PIDKinematicChainControllerBase() {}
		~PIDKinematicChainControllerBase() {}

        bool init(JI *robot, ros::NodeHandle &n);

	protected:
        using KinematicChainControllerBase<JI>::kdl_chain_;
        using KinematicChainControllerBase<JI>::joint_handles_;
        
        std::vector<control_toolbox::Pid> PIDs_;
        double Kp,Ki,Kd;
	};
    
    template <typename JI>
    bool PIDKinematicChainControllerBase<JI>::init(JI *robot, ros::NodeHandle &n)
    {
        KinematicChainControllerBase<JI>::init(robot, n);
        
        PIDs_.resize(kdl_chain_.getNrOfJoints());
        
        // Parsing PID gains from YAML 
        std::string pid_ = ("pid_");
        for (int i = 0; i < joint_handles_.size(); ++i)
        {
            if (!PIDs_[i].init(ros::NodeHandle(n, pid_ + joint_handles_[i].getName())))
            {
                ROS_ERROR("Error initializing the PID for joint %d",i);
                return false;
            }
        }
    }

}

#endif