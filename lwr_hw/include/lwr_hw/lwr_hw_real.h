#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// fri remote 
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include "fri/friudp.h"
#include "fri/friremote.h"
#include <std_msgs/Bool.h>

namespace lwr_hw
{ 
  class LWRHWreal : public lwr_hw::LWRHW
  {
  public:
    LWRHWreal(ros::NodeHandle nh);
    bool start();
    bool read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);
    void stop();
    void set_mode();

    // low-level interface
    boost::shared_ptr<friRemote> device_;

    // FRI values
    FRI_QUALITY lastQuality_;
    FRI_CTRL lastCtrlScheme_;

  private:

    // Node handle
    ros::NodeHandle nh_;
    ros::Subscriber limits_sub_;

    // Parameters
    bool enforce_limits_;
    int port_;
    std::string hintToRemoteHost_;
  };

}

#endif