
#include <lwr_controllers/inverse_dynamics_controller.h>

#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>

namespace lwr_controllers
{

  InverseDynamicsController::InverseDynamicsController()
    : robot_description_("")
      ,root_name_("")
      ,tip_name_("")
      ,joint_names_()
      ,id_solver_(NULL)
      ,ext_wrenches_()
      ,joint_states_()
      ,torques_()
  {}

  InverseDynamicsController::~InverseDynamicsController()
  {
    ext_wrench_sub_.shutdown();
  }

  bool InverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

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

PLUGINLIB_EXPORT_CLASS(lwr_controllers::InverseDynamicsController, controller_interface::ControllerBase)

