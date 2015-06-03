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
	- __single_lwr_moveit__: the moveit configuration for your `single_lwr_robot` description.
	- [__single_lwr_launch__](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_launch): a launch interface to load different components and configuration of your setup be it real, simulation, moveit, visualization, etc.

For an example using two LWR 4+ arms and two Pisa/IIT SoftHands, see the [Vito robot](https://github.com/CentroEPiaggio/vito-robot).

## Install

ToDo: one should check the commands in the [`.travis.yaml`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/.travis.yml) file when that is working in Ubuntu 14.04.

The most critical that are not straight forward if you want to use the simulation environment are:
- [Gazebo4](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=4.0&cat=install) or higher
- Slightly modified `transmission_interface` package that allows robot composability in gazebo @ (forked) [ros_control](https://github.com/CentroEPiaggio/ros_control/tree/multi-robot-test)


(ToDo: perhaps the two sections below could be moved to the `lwr_hw` package.)
## What to do to run a real LWR 4+

1. Load the script [`lwr_hw/krl/ros_control.src`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_control.src) and the corresponding [`.dat`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_control.src) on the robot.  
2. Place the robot in a position where the joints 1 and 3 are as bent as possible (at least 45 degrees) to avoid the "__FRI interpolation error__". A good way to check this is to go to gravity compensation, and see if the robot enters succesfully in that mode. 
3. Set the robot in __Position__ control. 
4. Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button. You can also use the script in semi-automatic mode.  
5. Once the script reaches the loop where it waits for commands, and then, you can proceed to launch your hardware interface + controllers + anything on top of it, e.g. moveit.

## Gravity compensation
(ToDo: revise this procedure after the merge)

In this mode the robot can be moved manually, while the position of each joint is available in TF (__IMPORTANT__: due to the robot own software policies, this mode is only available in T1). This mode can be very useful for calibration, tracking, or teaching.  
1. Load the script [`lwr_hw/krl/ros_monitor.src`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_monitor.src) and the corresponding [`.dat`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_monitor.dat) on the robot.
2. Set the robot in __Position__ control (it will be changed to __Gravity Compensation__ inside the script).  
3. Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button (you can also use the script in automatic mode)  
4. The script should reach the loop where it waits for commands
5. Launch the hardware interface.
6. Start the publisher node with 'roslaunch lwr_launch state_publisher.launch'
