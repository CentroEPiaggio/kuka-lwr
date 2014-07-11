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

#include "inverse_dynamics_controllers/inverse_dynamics_controller.h"
#include <Eigen/Dense>

#include <pluginlib/class_list_macros.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace kuka_controllers
{

  InverseDynamicsController::InverseDynamicsController()
    : robot_description_("")
      ,root_name_("")
      ,tip_name_("")
      ,joint_names_()
      ,gravity_(KDL::Vector::Zero())
      ,n_dof_(0)
      ,kdl_tree_()
      ,kdl_chain_()
      ,id_solver_(NULL)
      ,ext_wrenches_()
      ,joint_states_()
      ,torques_()
      ,joint_handles_()
  {}

  InverseDynamicsController::~InverseDynamicsController()
  {
    ext_wrench_sub_.shutdown();
  }

  bool InverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    // Grab the handle
    nh_ = n;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description_, root_name_, tip_name_;

    if (!ros::param::search(n.getNamespace(),"robot_description", robot_description_)){
      ROS_ERROR_STREAM("InverseDynamicsController: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
      return false;
    }
    if (!nh_.getParam("root_name", root_name_)){
      ROS_ERROR_STREAM("InverseDynamicsController: No root link name found on parameter server ("<<n.getNamespace()<<"/root_name)");
      return false;
    }
    if (!nh_.getParam("tip_name", tip_name_)){
      ROS_ERROR_STREAM("InverseDynamicsController: No tip link name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
      return false;
    }

    // Get the gravity vector (direction and magnitude)
    gravity_ = KDL::Vector::Zero();
    XmlRpc::XmlRpcValue xml_gravity;
    n.getParam("gravity", xml_gravity);
    if(xml_gravity.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN_STREAM("Gravity list parameter not found under: "
                      <<n.getNamespace()<<"/gravity, assuming no gravity.");
    } else {
      for (int i = 0; i < 3; ++i) {
        ROS_ASSERT(xml_gravity[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        gravity_[i] = static_cast<double>(xml_gravity[i]);
      }
    }

     // Construct an URDF model from the xml string


	std::string xml_string;

	if (n.hasParam(robot_description_))
	{
	  n.getParam(robot_description_.c_str(), xml_string);
	}
    else
    {
	    ROS_ERROR("Parameter %s not set, shutting down node...", robot_description_.c_str());
	    n.shutdown();
	    return false;
	}

	if (xml_string.size() == 0)
	{
	    ROS_ERROR("Unable to load robot model from parameter %s",robot_description_.c_str());
	    n.shutdown();
	    return false;
	}

	  ROS_DEBUG("%s content\n%s", robot_description_.c_str(), xml_string.c_str());

    // Construct an URDF model from the xml string
    urdf::Model urdf_model;
    //urdf_model.initString(robot_description_);
    if (!urdf_model.initString(xml_string))
	{
		ROS_ERROR("Failed to parse urdf file");
		n.shutdown();
		return false;
	}
	ROS_INFO("Successfully parsed urdf file");

    // Get a KDL tree from the robot URDF
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
      ROS_ERROR("Failed to construct kdl tree from URDF model");
      return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name_, tip_name_, kdl_chain_))
    {
      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  "<<root_name_<<" --> "<<tip_name_);
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree.getSegments();
      KDL::SegmentMap::iterator it;

      for( it=segment_map.begin();
           it != segment_map.end();
           it++ )
      {
        ROS_ERROR_STREAM( "    "<<(*it).first);
      }

      return false;
    }

    // Get the number of joints
    n_dof_ = kdl_chain_.getNrOfJoints();

    // Get joint handles for all of the joints in the chain
    for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin()+1;
    it != kdl_chain_.segments.end(); ++it)
    {
      joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
      ROS_DEBUG("%s", it->getJoint().getName().c_str() );
    }

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );


    // ---- OLD ORIGINAL CODE, gave error when storing joint names
    /*
    //Get hardware joint names
    //std::vector<std::string> hw_joint_names = robot->getJointNames();
    // joint_names_.clear();
    // for(std::vector<KDL::Segment>::const_iterator segment = kdl_chain_.segments.begin();
    //     segment != kdl_chain_.segments.end();
    //     ++segment)
    // {
    //   // Store the joint name
    //   joint_names_.push_back(segment->getJoint().getName());

    //   // Check if this KDL joint is provided by the hw interface
    //   if(hw_joint_names.end() != std::find(hw_joint_names.begin(),
    //                                        hw_joint_names.end(),
    //                                        joint_names_.back()))
    //   {
    //     ROS_ERROR_STREAM("Joint \""<<joint_names_.back()
    //                      <<"\" in URDF not provided by this hardware interface.");
    //     return false;
    //   }

    //   // Get a joint handle for this joint
    //   joint_handles_.push_back(robot->getJointHandle(joint_names_.back()));
    // }*/

    //Create inverse dynamics solver
    id_solver_.reset( new KDL::ChainIdSolver_RNE( kdl_chain_, gravity_) );

    // Resize working vectors
    joint_states_.resize(n_dof_);
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

    // Zero out torque data
    torques_.data.setZero();

    // Subscribe to external wrench topic
    ext_wrench_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
        "ext_wrench", 1,
        &InverseDynamicsController::ext_wrench_cb, this);

    return true;
  }

  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Read the positions/velocities
    for(unsigned int j=0; j<n_dof_; j++) {
      joint_states_.q(j) = joint_handles_[j].getPosition();
      joint_states_.qdot(j) = joint_handles_[j].getVelocity();
      // JointStateInterface has no acceleration support 
      // joint_states_.qdotdot(j) = joint_handles_[j].getAcceleration();
      joint_states_.qdotdot(j) = 0.0;
    }

    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
    if(id_solver_->CartToJnt(
            joint_states_.q,
            joint_states_.qdot,
            joint_states_.qdotdot,
            ext_wrenches_,
            torques_) != 0)
    {
      ROS_ERROR("Could not compute joint torques! Setting all torques to zero!");
      KDL::SetToZero(torques_);
    }

    // Set the commands
    for(unsigned int j=0; j<n_dof_; j++) {
      	joint_handles_[j].setCommand(torques_(j));
    	//joint_handles_[j].setCommand(0);
    }
  }

  void InverseDynamicsController::ext_wrench_cb(
      const geometry_msgs::WrenchStampedConstPtr &wrench_msg)
  {
    // TODO: Transform wrench into appropriate frame (probably local frame for each link)

    // Convert to KDL 
    KDL::Wrench wrench(
        KDL::Vector(wrench_msg->wrench.force.x,
                    wrench_msg->wrench.force.y,
                    wrench_msg->wrench.force.z),
        KDL::Vector(wrench_msg->wrench.torque.x,
                    wrench_msg->wrench.torque.y,
                    wrench_msg->wrench.torque.z));

    // Set the wrench on the tip link
    ext_wrenches_.back() = wrench;
  }

}

PLUGINLIB_EXPORT_CLASS(kuka_controllers::InverseDynamicsController, controller_interface::ControllerBase)

