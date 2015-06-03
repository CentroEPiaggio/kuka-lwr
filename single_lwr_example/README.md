# single_lwr_example

This package is a guide-trough-example to show how to use the [kuka_lwr](https://github.com/CentroEPiaggio/kuka-lwr) packages.

## 1. Set your scenario

Create you robot and environment using the lwr model. 

For instance, in the package [__single_lwr_robot__](./single_lwr_robot/), the [robot](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/robot/single_lwr_robot.urdf.xacro) is a lwr mounted on a box, and the [environment](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/worlds/simple_environment.world) is just a ground plane with a sun.

[Controllers](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_moveit/config/controllers.yaml#L2) and [joint names](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/config/joint_names.yaml) are in the [config](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/single_lwr_example/single_lwr_robot/config) folder. Note that the name space of controllers and the joint names contain the word `lwr`, which is the name you gave to the arm when creating the robot [here](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/robot/single_lwr_robot.urdf.xacro#L36).

__NOTE__: Other than standard ros-controllers, there are custom controllers that can be found in the [lwr_controllers](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_controllers) package.

## 2. Configure MoveIt!

Use the setup assistant to configure a MoveIt! package of your robot.

We already did that in the package [__single_lwr_moveit__](./single_lwr_moveit/). We followed the instructions given in Section 3 [here](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot) to update the configuration files of MoveIt!.

Note that the [name of the controller](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_moveit/config/controllers.yaml#L2) and [joint names](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_moveit/config/controllers.yaml#L7-13) must coincide with the name given in the robot package for the [controller](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/config/controllers.yaml#L8) and [joint names](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/single_lwr_example/single_lwr_robot/config/joint_names.yaml).

__NOTE__: recall that to run a real LWR 4+, you must follow instructions in [lwr_hw](https://github.com/CentroEPiaggio/kuka-lwr/tree/master/lwr_hw).

## 3. Set your launch control panel

We encourage to have a separated package with your launch file to configure your different uses of your robot, including the network parameters.

We have an example in the package [__single_lwr_launch__](./single_lwr_launch). To check which arguments are available use:

`roslaunch --ros-args single_lwr_launch single_lwr.launch`

Argument autocomplete was available in Groovy, but somehow it skipped Hydro, hence Indigo, but it is comming, see [this issue](https://github.com/ros/ros_comm/issues/575).

## 4. Test in simulation

__a.__ Just bring your robot on:

`roslaunch single_lwr_launch single_lwr.launch`

By default, it does not load the MoveIt! configuration. Eventhough, you can open `rqt`, and open Plugins->Robot Tools->Joint trajectory controller and move the robot manually with slides.

__b.__ Testing MoveIt! the configuration in simulation:

`roslaunch single_lwr_launch single_lwr.launch load_moveit:=true`

## 5. Test in real

__a.__ Once everything went well in simulation, you can procede to use the real robot by using:

`roslaunch single_lwr_launch single_lwr.launch use_lwr_sim:=false lwr_powered:=true`

__b.__ If you wish to use MoveIt!, you can potentially use:

`roslaunch single_lwr_launch single_lwr.launch use_lwr_sim:=false lwr_powered:=true load_moveit:=true`

If the robot complains a lot about bad communication quality, then just launch the robot with the previous command, and after everything has started, then use:

`roslaunch single_lwr_moveit move_group.launch allow_trajectory_execution:=true fake_execution:=false info:=true debug:=false`

And that's it!  :metal:
