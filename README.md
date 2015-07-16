# KUKA LWR 4+

[![Build Status](https://api.travis-ci.org/CentroEPiaggio/kuka-lwr.svg)](https://travis-ci.org/CentroEPiaggio/kuka-lwr)

ROS (tested on indigo) packages to work with the KUKA LWR 4+.

## Overview
The main packages are:
- [__lwr_description__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_description): a package that defines the model of the robot (ToDo: name it __lwr_model__)
- [__lwr_hw__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_hw): a package that contains the LWR 4+ definition within the ros control framework, and also final interfaces using Kuka FRI, Stanford FRI Library or a Gazebo plugin. Read adding an interface below if you wish to add a different non-existing interface. 
- [__lwr_controllers__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_controllers): a package that implement a set of useful controllers (ToDo: perhaps moving this to a forked version of `ros_controllers` would be ok, but some controllers are specific for the a 7-dof arm).
- [__single_lwr_example__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example): a cofiguration-based meta-package that shows how to use the `kuka_lwr` packages.
	- [__single_lwr_robot__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_robot): the package where you define your robot using the LWR 4+ arm.
	- [__single_lwr_moveit__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_moveit): the moveit configuration for your `single_lwr_robot` description.
	- [__single_lwr_launch__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_launch): a launch interface to load different components and configuration of your setup be it real, simulation, moveit, visualization, etc.

For an example using two LWR 4+ arms and two Pisa/IIT SoftHands, see the [Vito robot](https://github.com/CentroEPiaggio/vito-robot).

## Install

Check the [`.travis.yaml`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/.travis.yml) file, that'll give you the basic steps to install all necessary packages to compile the package.

## Adding more interfaces/platforms

The package [lwr_hw](lwr_hw) contains the abstraction that allows to make the most of the new ros control framework. The abstraction is enforced with [three pure virtual functions](lwr_hw/include/lwr_hw/lwr_hw.h#L61-L64):
``` c++
  virtual bool init() = 0;
  virtual void read(ros::Time time, ros::Duration period) = 0;
  virtual void write(ros::Time time, ros::Duration period) = 0;
```

Adding an interface boils down to inherit from the [LWRHW class](lwr_hw/include/lwr_hw/lwr_hw.h#L33), implement these three function according to your final platform, and creating either a node or a plugin that uses your new class. This way you can use all your planning and controllers setup in any final real or simulated robot.

Examples of final interface class implementations are found for the [Kuka FRI](lwr_hw/include/lwr_hw/lwr_hw_fri.hpp), [Stanford FRI library](lwr_hw/include/lwr_hw/lwr_hw_fril.hpp) and a [Gazebo simulation](lwr_hw/include/lwr_hw/lwr_hw_gazebo.hpp). The corresponding nodes and plugin are found [here](lwr_hw/src/lwr_hw_fri_node.cpp), [here](lwr_hw/src/lwr_hw_fril_node.cpp), and [here](lwr_hw/src/lwr_hw_gazebo_plugin.cpp).

## Using the Stanford FRI Library

You need to provide your user name with real time priority and memlock limits higher than the default ones. You can do it permanently like this:

1. `sudo nano /etc/security/limits.conf` and add these lines: 
```
YOUR_USERNAME hard rtprio 95
YOUR_USERNAME soft rtprio 95
YOUR_USERNAME hard memlock unlimited
YOUR_USERNAME soft memlock unlimited
```
2. `sudo  nano /etc/pam.d/common-session` and add `session required pam_limits.so`
3. Reboot, open a terminal, and check that `ulimit -r -l` gives you the values set above.
4. You need to edit manually the [lwr_hw.launch](lwr_hw/launch/lwr_hw.launch)
5. Load the KRL script that is downloaded with the library to the Robot, and follow the instructions from the [original link](http://cs.stanford.edu/people/tkr/fri/html/) to set up network and other requirements properly.
