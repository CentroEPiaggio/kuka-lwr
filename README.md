# KUKA LWR 4+

[![Build Status](https://api.travis-ci.org/CentroEPiaggio/kuka-lwr.svg)](https://travis-ci.org/CentroEPiaggio/kuka-lwr) (<- waiting for Ubuntu 14.04 support)

ROS indigo metapackage that contains packages to work with the KUKA LWR 4+.

## Overview
The main packages are:
- [__lwr_description__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_description): a package that defines the model of the robot (ToDo: name it __lwr_model__)
- [__lwr_hw__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_hw): a package that allows communication with an LWR 4+ through FRI or through a gazebo simulation. If you are using the joint impedance control strategy, it adds the gravity term computed from the URDF model.
- [__lwr_controllers__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_controllers): a package that implement a set of useful controllers (ToDo: perhaps moving this to a forked version of `ros_controllers` would be ok, but some controllers are specific for the a 7-dof arm).
- [__single_lwr_example__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example): a cofiguration-based meta-package that shows how to use the `kuka_lwr` packages.
	- [__single_lwr_robot__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_robot): the package where you define your robot using the LWR 4+ arm.
	- [__single_lwr_moveit__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_moveit): the moveit configuration for your `single_lwr_robot` description.
	- [__single_lwr_launch__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_launch): a launch interface to load different components and configuration of your setup be it real, simulation, moveit, visualization, etc.

For an example using two LWR 4+ arms and two Pisa/IIT SoftHands, see the [Vito robot](https://github.com/CentroEPiaggio/vito-robot).

## Install

ToDo: one should check the commands in the [`.travis.yaml`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/.travis.yml) file when that is working in Ubuntu 14.04.

The most critical that are not straight forward if you want to use the simulation environment are:
- [Gazebo4](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=4.0&cat=install) or higher
- Slightly modified `transmission_interface` package that allows robot composability in gazebo @ (forked) [ros_control](https://github.com/CentroEPiaggio/ros_control/tree/multi-robot-test)
