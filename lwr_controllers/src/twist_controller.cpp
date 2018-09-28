#include <lwr_controllers/twist_controller.h>

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
  bool TwistController::init(hardware_interface::PositionJointInterface *robot,
    ros::NodeHandle &n){

  }

  // Starting Function
  void TwistController::starting(const ros::Time& time){

  }

  // Updating Function
  void TwistController::update(const ros::Time& time,
    const ros::Duration& period){

  }

  // Command Topic Callback Function
  void TwistController::command(const lwr_controllers::PoseRPY::ConstPtr &msg){

  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::TwistController, controller_interface::ControllerBase)
