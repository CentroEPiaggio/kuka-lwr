// SYS
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

// the lwr hw real interface
#include "lwr_hw/lwr_hw_real.hpp"

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

int main( int argc, char** argv )
{
  // initialize ROS
  ros::init(argc, argv, "lwr_hw_interface", ros::init_options::NoSigintHandler);

  // ros spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // create a node
  ros::NodeHandle lwr_nh("");

  // get params or give default values
  int port;
  std::string hintToRemoteHost;
  lwr_nh.param("port", port, 49939);
  lwr_nh.param("ip", hintToRemoteHost, std::string("192.168.0.10") );

  // construct and start the real lwr
  lwr_hw::LWRHWreal lwr_robot;
  lwr_robot.create();
  lwr_robot.setPort(port);
  lwr_robot.setIP(hintToRemoteHost);
  if(!lwr_robot.init())
  {
    ROS_FATAL_NAMED("lwr_hw","Could not initialize robot real interface");
    return -1;
  }

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  //the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);

  // run as fast as possible
  while( !g_quit ) 
  {
    // get the time / period
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
    lwr_robot.read(now, period);

    // update the controllers
    manager.update(now, period);

    // write the command to the lwr
    lwr_robot.write(now, period);
  }

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping LWR..."<<std::endl;
  lwr_robot.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}