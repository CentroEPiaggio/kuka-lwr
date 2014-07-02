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

// from terse_roscpp for parameters, ToDo: avoid this
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
    bool start();
    bool read();
    void write();
    void stop();

    // Wait for all devices to become active
    bool wait_for_mode(
        ros::Duration timeout = ros::Duration(60.0),
        ros::Duration poll_duration = ros::Duration(0.1) );

    void set_mode();

    // Structure for a lwr, joint handles, low lever interface, etc
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

      // State and commands
      Eigen::Matrix<double,7,1> 
        joint_positions,
        joint_velocities,
        joint_efforts,
        joint_positions_cmds,
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

    // Node handle
    ros::NodeHandle nh_;

    // Configuration
    urdf::Model urdf_model_;
    int port_;
    std::string hintToRemoteHost_;

    // ros-controls hardware interfaces
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::EffortJointInterface effort_interface_;

  protected:

  };

  LWRHW::LWRHW(ros::NodeHandle nh) :
    nh_(nh)
  {}

  bool LWRHW::start()
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

    // get URDF links starting at product root link
    std::string tip_joint_name;
    param::require(nh_,"tip_joint",tip_joint_name, "LWR tip joint name in URDF.");
    boost::shared_ptr<const urdf::Joint> joint = urdf_model_.getJoint(tip_joint_name);
    if(!joint.get())
    {
      ROS_ERROR_STREAM("Tip joint does not exist: "<<tip_joint_name);
      throw std::runtime_error("Ran out of joints.");
    }

    // resize joint names
    this->device_->joint_names.resize(7);

    // create joint handles starting at the tip
    for(int i=7-1; i>=0; i--)
    {

        ROS_INFO_STREAM("Handling joint: "<<joint->name);

        // store the joint name
        this->device_->joint_names[i] = joint->name;
        //this->device_->position_limits(i) = joint->limits->JointLimits;
        this->device_->effort_limits(i) = joint->limits->effort;
        this->device_->velocity_limits(i) = joint->limits->velocity;

        // joint state handle
        hardware_interface::JointStateHandle state_handle(joint->name,
            &this->device_->joint_positions(i),
            &this->device_->joint_velocities(i),
            &this->device_->joint_effort_cmds(i));

        state_interface_.registerHandle(state_handle);

        // effort command handle
        effort_interface_.registerHandle(
            hardware_interface::JointHandle(
              state_interface_.getHandle(joint->name),
              &this->device_->joint_effort_cmds(i)));

        joint = urdf_model_.getLink(joint->parent_link_name)->parent_joint;
        // make sure we didn't run out of links
        if(!joint.get())
        {
          ROS_ERROR_STREAM("Ran out of joints while parsing URDF starting at joint: "<<tip_joint_name);
          throw std::runtime_error("Ran out of joints.");
        }
    }

    ROS_INFO("Register state and effort interfaces");

    // register ros-controls interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&effort_interface_);

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

    ROS_INFO_STREAM("LWR Status:\n" << this->device_->interface->getMsrBuf().intf);

    this->device_->interface->doDataExchange();
    ROS_INFO("Done handshake !");
    return true;
  }

  void LWRHW::stop()
  {
    // TODO: decide whether to stop the FRI or just put to idle
  }

  bool LWRHW::read()
    {
      // update the robot positions
      for (int i = 0; i < LBR_MNJ; i++)
      {
        this->device_->joint_positions[i] = this->device_->interface->getMsrMsrJntPosition()[i];
        this->device_->joint_efforts[i] = this->device_->interface->getMsrJntTrq()[i];
      }
      
      this->device_->interface->doDataExchange();

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

  void LWRHW::write()
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
      //float newJntVals[LBR_MNJ];
      //this->device_->interface->doPositionControl(newJntVals);

      // Stop request is issued from the other side
      if ( this->device_->interface->getFrmKRLInt(0) == -1)
      {
          ROS_INFO(" Stop request issued from the other side");
          this->stop();
      }

      // Quality change leads to output of statistics
      // for informational reasons
      //
      if ( this->device_->interface->getQuality() != this->device_->lastQuality )
      {
          ROS_INFO_STREAM("Quality change detected "<< this->device_->interface->getQuality()<< " \n");
          ROS_INFO_STREAM("" << this->device_->interface->getMsrBuf().intf);
          this->device_->lastQuality = this->device_->interface->getQuality();
      }

      this->device_->interface->doDataExchange();
      return;
    }

  bool LWRHW::wait_for_mode(
      ros::Duration timeout, 
      ros::Duration poll_duration) 
  {
    // ToDo
    return true;
  }

  void LWRHW::set_mode()
  {
    // ToDo: just switch between monitor and command mode
    // not control strategies!
      return;
  }

}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lwr_server", ros::init_options::NoSigintHandler);

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Construct the lwr structure
  ros::NodeHandle lwr_nh("");

  lwr_ros_control::LWRHW lwr_robot(lwr_nh);

  // configuration routines
  lwr_robot.start();

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(lwr_nh, "loop_rate", 2);

  //the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);

  uint32_t count = 0;

  // Run as fast as possible
  while( !g_quit ) 
  {
    // Get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) 
    {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else 
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // read the state from the lwr
    if(!lwr_robot.read())
    {
      g_quit = true;
      break;
    }

    // Update the controllers
    manager.update(now, period);

    // Write the command to the lwr
    lwr_robot.write();

    // if(count++ > 1000)
    // {
    //   if(publisher.trylock())
    //   {
    //     count = 0;
    //     publisher.msg_.data = period;
    //     publisher.unlockAndPublish();
    //   }
    // }
  }

  //publisher.stop();

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping LWR..."<<std::endl;
  lwr_robot.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}
