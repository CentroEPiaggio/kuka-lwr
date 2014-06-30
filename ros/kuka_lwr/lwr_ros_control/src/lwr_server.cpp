#include <ros/ros.h>
//#include <native/task.h>
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <controller_manager/controller_manager.h>
#include <signal.h>
#include <realtime_tools/realtime_publisher.h>
#include <control_toolbox/filters.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Duration.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>

#include <Eigen/Dense>

// fri remote 
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include "friudp.h"
#include "friremote.h"

// from terse_roscpp
#include <param.h>

#include <stdexcept>

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

namespace lwr_ros_control
{ 
  class LWRHW : public hardware_interface::RobotHW
  {
  public:
    LWRHW(ros::NodeHandle nh);
    bool configure();
    bool start();
    bool read(const ros::Time time, const ros::Duration period); 
    void write(const ros::Time time, const ros::Duration period);
    void stop();

    // Wait for all devices to become active
    bool wait_for_mode(/*barrett::SafetyModule::SafetyMode mode,*/
        ros::Duration timeout = ros::Duration(60.0), 
        ros::Duration poll_duration = ros::Duration(0.1) );

    void set_mode(/*barrett::SafetyModule::SafetyMode mode*/);

    // State structure for a lwr
    // This provides storage for the joint handles
    struct LWRDevice7
    {
      // Low-level interface
      boost::shared_ptr<friRemote> interface;

      // Configuration
      std::vector<std::string> joint_names;

      Eigen::Matrix<double,7,1> 
        position_limits,
        effort_limits,
        velocity_limits;

      // State
      Eigen::Matrix<double,7,1> 
        joint_positions,
        joint_velocities,
        joint_effort_cmds;

      // FRI values
      FRI_QUALITY lastQuality;
      FRI_CTRL lastCtrlScheme;

      void set_zero() 
      {
        joint_positions.setZero();
        joint_velocities.setZero();
        joint_effort_cmds.setZero();
      }

    };

    boost::shared_ptr<LWRHW::LWRDevice7> device_;

  private:

    // State
    ros::NodeHandle nh_;
    bool configured_;

    // Configuration
    urdf::Model urdf_model_;
    int port_;
    std::string hintToRemoteHost_;

    // ros-controls interface
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::EffortJointInterface effort_interface_;

  protected:

  };

  LWRHW::LWRHW(ros::NodeHandle nh) :
    nh_(nh),
    configured_(false)
  {
    // TODO: Determine pre-existing calibration from ROS parameter server
  }

  bool LWRHW::configure() 
  {
    using namespace terse_roscpp;
    
    // Get URDF
    std::string urdf_str;
    param::require(nh_, "robot_description", urdf_str, "The URDF of the lwr arm.");
    urdf_model_.initString(urdf_str);

    param::require(nh_, "port", port_, "The port of the robot");
    param::require(nh_, "ip", hintToRemoteHost_, "The ip of the robot");

    // Construct a new lwr device (interface and state storage)
    this->device_.reset( new LWRHW::LWRDevice7() );

    // Construct a low-level lwr
    this->device_->interface.reset( new friRemote( port_, hintToRemoteHost_.c_str() ) );

    // Initialize FRI values
    this->device_->lastQuality = FRI_QUALITY_BAD;
    this->device_->lastCtrlScheme = FRI_CTRL_OTHER;

    // Initialize joint position limits
    //this->device_->position_limits = this->compute_resolver_ranges<DOF>(this->device_->interface);

    // Get URDF links starting at product root link
    std::string tip_joint_name;
    param::require(nh_,"tip_joint",tip_joint_name, "LWR tip joint name in URDF.");
    boost::shared_ptr<const urdf::Joint> joint = urdf_model_.getJoint(tip_joint_name);
    if(!joint.get())
    {
      ROS_ERROR_STREAM("Tip joint does not exist: "<<tip_joint_name);
      throw std::runtime_error("Ran out of joints.");
    }

    // Resize joint names
    this->device_->joint_names.resize(7);

    // Create joint handles starting at the tip
    for(int i=7-1; i>=0; i--)
    {

        ROS_INFO_STREAM("Handling joint: "<<joint->name);

        // Store the joint name
        this->device_->joint_names[i] = joint->name;
        //this->device_->position_limits(i) = joint->limits->JointLimits;
        this->device_->effort_limits(i) = joint->limits->effort;
        this->device_->velocity_limits(i) = joint->limits->velocity;

        // Joint State Handle
        hardware_interface::JointStateHandle state_handle(joint->name,
            &this->device_->joint_positions(i),
            &this->device_->joint_velocities(i),
            &this->device_->joint_effort_cmds(i));

        state_interface_.registerHandle(state_handle);

        // Effort Command Handle
        effort_interface_.registerHandle(
            hardware_interface::JointHandle(
              state_interface_.getHandle(joint->name),
              &this->device_->joint_effort_cmds(i)));

        //while(std::find(this->device_->joint_names.begin(), this->device_->joint_names.end(),joint->name) != this->device_->joint_names.end()
        //            || joint->type != urdf::Joint::REVOLUTE)
        //{
            // Get the next joint
            joint = urdf_model_.getLink(joint->parent_link_name)->parent_joint;
            // Make sure we didn't run out of links
            if(!joint.get())
            {
              ROS_ERROR_STREAM("Ran out of joints while parsing URDF starting at joint: "<<tip_joint_name);
              throw std::runtime_error("Ran out of joints.");
            }
        //}
    }

    ROS_INFO("Register state and effort interfaces");

    // Register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&effort_interface_);

    // Set configured flag
    configured_ = true;

    return true;
  }

  bool LWRHW::start()
  {
    // Guard on configured
    if(!configured_) {
      ROS_ERROR("LWR hardware must be configured before it can be started.");
      return false;
    }

    std::cout << "Opening FRI Version " 
      << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR 
      << " Interface for LWR ROS server" << std::endl;

    ROS_INFO("Performing handshake to KRL");

    // perform some arbitrary handshake to KRL -- possible in monitor mode already
    // send to krl int a value
    this->device_->interface->setToKRLInt(0,1);
    if ( this->device_->interface->getQuality() >= FRI_QUALITY_OK)
    {
        // send a second marker
        this->device_->interface->setToKRLInt(0,10);
    }

    //
    // just mirror the real value..
    //
    this->device_->interface->setToKRLReal(0,this->device_->interface->getFrmKRLReal(1));

    return true;
  }

  void LWRHW::stop()
  {
    // Set the mode to IDLE
    //this->set_mode(barrett::SafetyModule::IDLE);
    // Wait for the system to become active
    //this->wait_for_mode(barrett::SafetyModule::IDLE);
  }

  bool LWRHW::read(
        const ros::Time time, 
        const ros::Duration period)
    {
      // Get raw state
      Eigen::Matrix<double,7,1> raw_positions;

        // Prepare a new position command - if we are in command mode
      float newJntVals[LBR_MNJ];
      for (int i = 0; i < LBR_MNJ; i++)
      {
        raw_positions[i] = this->device_->interface->getMsrCmdJntPosition()[i];
      }
      
      // Store position
      this->device_->joint_positions = raw_positions;

      //Eigen::Matrix<double,7,1> raw_velocities = device->interface->getJointVelocities();

      // Smooth velocity 
      // TODO: parameterize time constant
      // for(size_t i=0; i<7; i++) 
      // {
      //   this->device_->joint_velocities(i) = filters::exponentialSmoothing(
      //       raw_velocities(i),
      //       device->joint_velocities(i),
      //       0.5);
      // }

      return true;
    }

  void LWRHW::write(
        const ros::Time time, 
        const ros::Duration period)
    {
      static int warning = 0;

      /*for(size_t i=0; i<DOF; i++) {
        if(std::abs(device->joint_effort_cmds(i)) > device->effort_limits[i]) {
          if(warning++ > 1000) {
            ROS_WARN_STREAM("Commanded torque ("<<device->joint_effort_cmds(i)<<") of joint ("<<i<<") exceeded safety limits! They have been truncated to: +/- "<<device->effort_limits[i]);
            warning = 0;
          }
          // Truncate this joint torque
          device->joint_effort_cmds(i) = std::max(
              std::min(device->joint_effort_cmds(i), device->effort_limits[i]),
              -1.0*device->effort_limits[i]);
        }
      }

      // Set the torques
      device->interface->setTorques(device->joint_effort_cmds);*/

      // if position control
      // Call to data exchange - and the like 
      //this->device_->interface->doPositionControl(newJntVals);

      // Stop request is issued from the other side
      if ( this->device_->interface->getFrmKRLInt(0) == -1)
      {
          ROS_INFO(" Stop request issued from the other side");
          this->stop();
      }
      //
      // Quality change leads to output of statistics
      // for informational reasons
      //
      if ( this->device_->interface->getQuality() != this->device_->lastQuality )
      {
          ROS_INFO_STREAM("Quality change detected "<< this->device_->interface->getQuality()<< " \n");
          ROS_INFO_STREAM("" << this->device_->interface->getMsrBuf().intf);
          this->device_->lastQuality = this->device_->interface->getQuality();
      }

      return;
    }

  bool LWRHW::wait_for_mode(
      ros::Duration timeout, 
      ros::Duration poll_duration) 
  {
    // ros::Time polling_start_time = ros::Time::now();
    // ros::Rate poll_rate(1.0/poll_duration.toSec());

    // for(ManagerMap::iterator it = barrett_managers_.begin(); 
    //     it != barrett_managers_.end();
    //     ++it) 
    // {
    //   while( ros::ok() 
    //       && (ros::Time::now() - polling_start_time < timeout)
    //       && (it->second->getSafetyModule()->getMode() != mode)) 
    //   {
    //     poll_rate.sleep();
    //   }
    // }
    // return (ros::Time::now() - polling_start_time < timeout);

    return true;
  }

  void LWRHW::set_mode(/*barrett::SafetyModule::SafetyMode mode*/)
  {
    //this->device_->interface->getState();
  }

}

int main( int argc, char** argv ){

  // Set up real-time task
  //mlockall(MCL_CURRENT | MCL_FUTURE);
  //RT_TASK task;
  //rt_task_shadow( &task, "GroupLWR", 99, 0 );

  // Initialize ROS
  ros::init(argc, argv, "lwr_server", ros::init_options::NoSigintHandler);

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Construct the lwr structure
  ros::NodeHandle lwr_nh("");

  lwr_ros_control::LWRHW lwr_robot(lwr_nh);
  //lwr_robot.configure();
  //lwr_robot.start();


  // Timer variables
  struct timespec ts = {0,0};

  // if(clock_gettime(CLOCK_REALTIME, &ts) != 0) {
  //   ROS_FATAL("Failed to poll realtime clock!");
  // }

  ros::Time 
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(lwr_nh, "loop_rate", 2);

  bool lwr_ok = false;
  while(!g_quit && !lwr_ok) {
    if(!lwr_robot.configure()) {
      ROS_ERROR("Could not configure LWR!");
    } else if(!lwr_robot.start()) {
      ROS_ERROR("Could not start LWR!");
    } else {
      ros::Duration(1.0).sleep();

      if(!lwr_robot.read(now, period)) {
        ROS_ERROR("Could not read from LWR!");
      } else {
        lwr_ok = true;
      }
    }

    ros::Duration(1.0).sleep();
  }

  // Construct the controller manager
  ros::NodeHandle nh;
  controller_manager::ControllerManager manager(&lwr_robot, nh);

  uint32_t count = 0;

  // Run as fast as possible
  while( !g_quit ) {
    // Get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // Read the state from the lwr
    if(!lwr_robot.read(now, period)) {
      g_quit=true;
      break;
    }

    // Update the controllers
    manager.update(now, period);

    // Write the command to the lwr
    lwr_robot.write(now, period);

    if(count++ > 1000)
    {
      if(publisher.trylock())
      {
        count = 0;
        publisher.msg_.data = period;
        publisher.unlockAndPublish();
      }
    }
  }

  publisher.stop();

  std::cerr<<"Stpping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping LWR..."<<std::endl;
  lwr_robot.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}
