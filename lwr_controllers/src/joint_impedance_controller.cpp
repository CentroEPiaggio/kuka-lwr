#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/joint_impedance_controller.h>

namespace lwr_controllers {

JointImpedanceController::JointImpedanceController() {}

JointImpedanceController::~JointImpedanceController() {}

bool JointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  PIDKinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

  K_.resize(kdl_chain_.getNrOfJoints());
  D_.resize(kdl_chain_.getNrOfJoints());

  ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

  for (int i = 0; i < joint_handles_.size(); ++i){
    if ( !nh_.getParam("stiffness_gains", K_(i) ) ){
      ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
      }
    }
  for (int i = 0; i < joint_handles_.size(); ++i){
    if ( !nh_.getParam("damping_gains", D_(i)) ){
      ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
      }
    }



  id_solver_gravity_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

  dotq_msr_.resize(kdl_chain_.getNrOfJoints());
  q_msr_.resize(kdl_chain_.getNrOfJoints());
  q_des_.resize(kdl_chain_.getNrOfJoints());
  tau_des_.resize(kdl_chain_.getNrOfJoints());
  tau_cmd_.resize(kdl_chain_.getNrOfJoints());
  tau_gravity_.resize(kdl_chain_.getNrOfJoints());
  // K_.resize(kdl_chain_.getNrOfJoints());
  // D_.resize(kdl_chain_.getNrOfJoints());

  sub_gains_ = nh_.subscribe("gains", 1, &JointImpedanceController::setGains, this);
  sub_posture_ = nh_.subscribe("command", 1, &JointImpedanceController::command, this);


  return true;


}

void JointImpedanceController::starting(const ros::Time& time)
{
  // get joint positions
  for(size_t i=0; i<joint_handles_.size(); i++) {
    K_(i) = 300.0;
    D_(i) = 0.7;
    tau_des_(i) = 0.0;
    dotq_msr_.q(i) = joint_handles_[i].getPosition();
    q_des_(i) = dotq_msr_.q(i);
    dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
    }


}

void JointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{

  // get joint positions	
  for(size_t i=0; i<joint_handles_.size(); i++) {
    dotq_msr_.q(i) = joint_handles_[i].getPosition();
    q_msr_(i) = dotq_msr_.q(i);
    dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
    }

  //Compute control law
  id_solver_gravity_->JntToGravity( q_msr_ , tau_gravity_ );
  for(size_t i=0; i<joint_handles_.size(); i++) {
    tau_cmd_(i) = K_(i) * (q_des_(i) - q_msr_(i)) + D_(i)*dotq_msr_.qdot(i) + tau_des_(i) + tau_gravity_(i);
    joint_handles_[i].setCommand(tau_cmd_	(i));
    }	

}


void JointImpedanceController::command(const std_msgs::Float64MultiArray::ConstPtr &msg){
  if (msg->data.size() == 0) {
    ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
  else if ((int)msg->data.size() != joint_handles_.size()) {
    ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
    return;
    }
  else
  {
    for (unsigned int j = 0; j < joint_handles_.size(); ++j)
    q_des_(j) = msg->data[j];
  }

  }

void JointImpedanceController::setGains(const std_msgs::Float64MultiArray::ConstPtr &msg){

  if (msg->data.size() == 2*joint_handles_.size()){
    for (unsigned int i = 0; i < joint_handles_.size(); ++i){
      K_(i) = msg->data[i];
      D_(i)= msg->data[i + joint_handles_.size()];
      }
    // for (unsigned int i = joint_handles_.size(); i < 2*joint_handles_.size(); ++i){
      // 	D_(i)= msg->data[i];
      // }
    }
  else
  {
    ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
  }

  ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

  ROS_INFO("New gains K: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  K_(0), K_(1), K_(2), K_(3), K_(4), K_(5), K_(6));
  ROS_INFO("New gains D: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  D_(0), D_(1), D_(2), D_(3), D_(4), D_(5), D_(6));

  }


}                                                           // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointImpedanceController, controller_interface::ControllerBase)
