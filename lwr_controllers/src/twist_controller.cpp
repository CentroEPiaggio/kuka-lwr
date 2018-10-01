#include <lwr_controllers/twist_controller.h>
#include <pluginlib/class_list_macros.h>
#include "kdl/chainfksolvervel_recursive.hpp"

namespace lwr_controllers {

  // Default Constructor
  TwistController::TwistController(){
    // Nothing to do
  }

  // Default Destructor
  TwistController::~TwistController(){
    // Nothing to do
  }

  // Initializing Function
  bool TwistController::init(hardware_interface::EffortJointInterface *robot,
    ros::NodeHandle &n){
    // Calling the Initializing function of KinematicChainControllerBase
    if(!(KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n))){
        ROS_ERROR("Couldn't initilize TwistController.");
        return false;
    }

    // Resetting the solvers
    jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resizing vectors and jacobian
    joint_states_.resize(kdl_chain_.getNrOfJoints());
    wrenches_.resize(kdl_chain_.getNrOfJoints());
    torques_.resize(kdl_chain_.getNrOfJoints());
    jacobian_.resize(kdl_chain_.getNrOfJoints());

    // Setting to zero the current effort
    torques_.data.setZero();

    // Create the PID controllers for following twist's x, y, z, r, p and y
    control_toolbox::Pid pid;
    if(!pid.init(ros::NodeHandle(n, "pid_trans"))){
      ROS_ERROR("Couldn't find PID translation gains.");
      return false;
    }
    for(unsigned int i = 0; i < 3; i++) pid_controller_.push_back(pid);

    if(!pid.init(ros::NodeHandle(n, "pid_rot"))){
      ROS_ERROR("Couldn't find PID rotation gains.");
      return false;
    }
    for(unsigned int i = 0; i < 3; i++) pid_controller_.push_back(pid);

    // Initializing subscriber to command topic
    sub_command_ = n.subscribe<geometry_msgs::Twist>("command", 1,
      &TwistController::command, this);

    return true;
  }

  // Starting Function
  void TwistController::starting(const ros::Time& time){
    // Resetting PID controllers
    for(unsigned int i = 0; i < 6; i++) pid_controller_[i].reset();

    // Getting previous time
    last_time_ = time;

    // Setting desired twist to zero
    twist_des_ = KDL::Twist::Zero();
  }

  // Updating Function
  void TwistController::update(const ros::Time& time,
    const ros::Duration& period){
    // Getting current time resolution
  }

  // Command Topic Callback Function
  void TwistController::command(const geometry_msgs::Twist::ConstPtr &msg){
    // Converting and saving msg to twist_des_
    twist_des_.vel(0) = msg->linear.x;
    twist_des_.vel(1) = msg->linear.y;
    twist_des_.vel(2) = msg->linear.z;
    twist_des_.rot(0) = msg->angular.x;
    twist_des_.rot(1) = msg->angular.y;
    twist_des_.rot(2) = msg->angular.z;
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::TwistController, controller_interface::ControllerBase)
