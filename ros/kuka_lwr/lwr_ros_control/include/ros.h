#ifndef __TERSE_ROSCPP_ROS_H
#define __TERSE_ROSCPP_ROS_H

#include <ros.h>
#include <param.h>

namespace terse_roscpp {
  class NodeHandle : public ros::NodeHandle {

    NodeHandle (const std::string &ns=std::string(), const M_string &remappings=M_string()) 
      : ros::NodeHandle(ns, remappings)
    {  }
    NodeHandle (const NodeHandle &rhs)
      : ros::NodeHandle(rhs)
    { }
    NodeHandle (const NodeHandle &parent, const std::string &ns) 
      : ros::NodeHandle(parent, ns)
    { }
    NodeHandle (const NodeHandle &parent, const std::string &ns, const M_string &remappings)
      : ros::NodeHandle(parent, ns, remappings)
    { }

    template<class T>
    bool getParam(const std::string &param_name, T &val) {
      return terse_roscpp::param::get<T>(*this, param_name, val, "N/A", false);
    }

    template<class T>
    bool requireParam(const std::string &param_name, T &val) {
      return terse_roscpp::param::get<T>(*this, param_name, val, "N/A", true);
    }
  }
}

#endif // ifndef __TERSE_ROSCPP_ROS_H