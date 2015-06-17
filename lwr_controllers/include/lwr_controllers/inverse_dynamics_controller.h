#ifndef LWR_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H
#define LWR_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <geometry_msgs/WrenchStamped.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>

namespace lwr_controllers
{

  class InverseDynamicsController : public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
  {
  public:

    InverseDynamicsController();
    ~InverseDynamicsController();

    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time& time) 
    {
      KDL::SetToZero(torques_);
    }

    void update(const ros::Time& time, const ros::Duration& period);

  private:
    ros::Subscriber ext_wrench_sub_;
    void ext_wrench_cb(const geometry_msgs::WrenchStampedConstPtr &wrench_msg);

    std::string 
      robot_description_,
      root_name_, 
      tip_name_;
    std::vector<std::string> joint_names_;
      

    unsigned int n_dof_;
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

    KDL::Wrenches ext_wrenches_;
    KDL::JntArrayAcc joint_states_;
    KDL::JntArray torques_;

  };

}

#endif
