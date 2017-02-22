
#include <lwr_controllers/KinematicChainControllerBase.h>
namespace controller_interface
{
template<>
bool KinematicChainControllerBase<hardware_interface::EffortJointInterface>::getHandles(hardware_interface::EffortJointInterface *robot)
{
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
    {
    	if ( it->getJoint().getType() != KDL::Joint::None )
        {
	        joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
	        joint_stiffness_handles_.push_back(robot->getHandle(it->getJoint().getName() + std::string("_stiffness")));
	        joint_damping_handles_.push_back(robot->getHandle(it->getJoint().getName() + std::string("_damping")));
	        joint_set_point_handles_.push_back(robot->getHandle(it->getJoint().getName() + std::string("_set_point")));
	        ROS_DEBUG("%s", it->getJoint().getName().c_str() );
	    }
    }

    // Inertia matrix handles
    int n_joints = 7;
    for (int i=0; i<n_joints; i++)
      for (int j=0; j<n_joints; j++)
	inertia_matrix_handles_.push_back(robot->getHandle("inertia_" + std::to_string(i) + "_" + std::to_string(j)));

    return true;
}
}

