#ifndef KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include <control_msgs/JointControllerState.h> // TODO: state message for all controllers?

#include <urdf/model.h>
#include <lwr_hw/lwr_hw.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>

namespace controller_interface
{
    template<typename JI>
	class KinematicChainControllerBase: public Controller<JI>
	{
	public:
		KinematicChainControllerBase() {}
		~KinematicChainControllerBase() {}

        bool init(JI *robot, ros::NodeHandle &n);

	protected:
		ros::NodeHandle nh_;

		KDL::Chain kdl_chain_;
        KDL::Vector gravity_; 
        KDL::JntArrayAcc joint_msr_states_, joint_des_states_;  // joint states (measured and desired)

		struct limits_
		{
			KDL::JntArray min;
			KDL::JntArray max;
			KDL::JntArray center;
		} joint_limits_;

		std::vector<typename JI::ResourceHandleType> joint_handles_;
        std::vector<typename JI::ResourceHandleType> joint_stiffness_handles_;
        std::vector<typename JI::ResourceHandleType> joint_damping_handles_;
        std::vector<typename JI::ResourceHandleType> joint_set_point_handles_;
        
        bool getHandles(JI *robot);
        
	};
    
    template <typename JI>
    bool KinematicChainControllerBase<JI>::init(JI *robot, ros::NodeHandle &n)
    {
        nh_ = n;

        // get URDF and name of root and tip from the parameter server
        std::string robot_description, root_name, tip_name;

        if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
        {
            ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
            return false;
        }

        if (!nh_.getParam("root_name", root_name))
        {
            ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<n.getNamespace()<<"/root_name)");
            return false;
        }

        if (!nh_.getParam("tip_name", tip_name))
        {
            ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<n.getNamespace()<<"/tip_name)");
            return false;
        }
     
        // Get the gravity vector (direction and magnitude)
        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;

        // Construct an URDF model from the xml string
        std::string xml_string;

        if (n.hasParam(robot_description))
            n.getParam(robot_description.c_str(), xml_string);
        else
        {
            ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
            n.shutdown();
            return false;
        }

        if (xml_string.size() == 0)
        {
            ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
            n.shutdown();
            return false;
        }

        ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
        
        // Get urdf model out of robot_description
        urdf::Model model;
        if (!model.initString(xml_string))
        {
            ROS_ERROR("Failed to parse urdf file");
            n.shutdown();
            return false;
        }
        ROS_INFO("Successfully parsed urdf file");
        
        KDL::Tree kdl_tree_;
        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            n.shutdown();
            return false;
        }

        // Populate the KDL chain
        if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
            ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for( it=segment_map.begin(); it != segment_map.end(); it++ )
              ROS_ERROR_STREAM( "    "<<(*it).first);

            return false;
        }

        ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
        ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
        
        // Parsing joint limits from urdf model along kdl chain
        boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
        boost::shared_ptr<const urdf::Joint> joint_;
        joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
        joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
        joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
        int index;
        
        for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
        {
            joint_ = model.getJoint(link_->parent_joint->name);  
            ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
            index = kdl_chain_.getNrOfJoints() - i - 1;

            joint_limits_.min(index) = joint_->limits->lower;
            joint_limits_.max(index) = joint_->limits->upper;
            joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

            link_ = model.getLink(link_->getParent()->name);
        }

        // Get joint handles for all of the joints in the chain
        getHandles(robot);
        
        ROS_DEBUG("Number of joints in handle = %lu", joint_handles_.size() );
        
        joint_msr_states_.resize(kdl_chain_.getNrOfJoints());
        joint_des_states_.resize(kdl_chain_.getNrOfJoints());

        return true;
    }
    
    template <typename JI>
    bool KinematicChainControllerBase<JI>::getHandles(JI *robot)
    {
        for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        {
            if ( it->getJoint().getType() != KDL::Joint::None )
            {
                joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
                ROS_DEBUG("%s", it->getJoint().getName().c_str() );
            }
        }        
        return true;
    }
    
    
}

#endif