// SYS
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>

// the lwr hw real interface
#include "lwr_hw/lwr_hw_real.h"

bool g_quit = false;
volatile bool paused = false;

void quitRequested(int sig)
{
  g_quit = true;
}

void stopUpdatingController(const std_msgs::String::ConstPtr & msg)
{
  if(msg->data == "stop")
    paused = true;
  if(msg->data == "start")
    paused = false;
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

  // construct the lwr
  ros::NodeHandle lwr_nh("");
  lwr_hw::LWRHWreal lwr_robot(lwr_nh);
  
  ros::Subscriber s = lwr_nh.subscribe<std_msgs::String>("emergency_stop",10,&stopUpdatingController);

  // configuration routines
  lwr_robot.start();

  // timer variables
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  //realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(lwr_nh, "loop_rate", 2);

  //the controller manager
  controller_manager::ControllerManager manager(&lwr_robot, lwr_nh);

  //uint32_t count = 0;

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
    if(!lwr_robot.read(now, period))
    {
      g_quit = true;
      break;
    }

    // update the controllers
    if(!paused)
      manager.update(now, period);

    // write the command to the lwr
    lwr_robot.write(now, period);

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