#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "LWRSim.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwr_controller_manager");
  ros::NodeHandle nh;

  LWRSim robot;
  controller_manager::ControllerManager cm(&robot);

  while (true)
  {
     //robot.read();
     //cm.update(robot.get_time(), robot.get_period());
     //robot.write();
  }
}