/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef KUKA_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H
#define KUKA_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>


#include <kdl/jntarrayacc.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>



namespace kuka_controllers
{

  class InverseDynamicsController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
    ros::NodeHandle nh_;
    ros::Subscriber ext_wrench_sub_;
    void ext_wrench_cb(const geometry_msgs::WrenchStampedConstPtr &wrench_msg);

    std::string 
      robot_description_,
      root_name_, 
      tip_name_;
    std::vector<std::string> joint_names_;
      
    KDL::Vector gravity_;

    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

    KDL::Wrenches ext_wrenches_;
    KDL::JntArrayAcc joint_states_;
    KDL::JntArray torques_;

    std::vector<hardware_interface::JointHandle> joint_handles_;
  };

}

#endif
