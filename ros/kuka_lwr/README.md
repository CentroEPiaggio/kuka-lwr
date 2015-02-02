# KUKA LWR

ROS indigo metapackage that contains ROS related code to work with the KUKA LWR 4+

## Overview
This repository contains:
- __lwr_hw__: a package that allows communication with an LWR 4+ through FRI
- __lwr_controllers__: implementation of a series of advanced controlling strategies
- __lwr_launch__: launch files for these advanced controllers (to control either a Gazebo simulation or the real hardware)
- __lwr_moveit__: a MoveIt! configuration for controlling the robot (either a Gazebo simulation or the real hardware)

__IMPORTANT__: this repository is to be considered experimental! Please do not try the MoveIt! package yet with a real robot, as the PID gains are under tuning. The OneTask- and MultiTaskInverseKinematics controllers are already in a working condition.

## Dependencies

For control and simulation (_lwr_controllers_):

`sudo apt-get install gazebo ros-indigo-gazebo-ros-control ros-indigo-ros-control ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-joint-trajectory-controller ros-indigo-joint-trajectory-action`

For planning (_lwr_moveit_):

`sudo apt-get install ros-indigo-moveit-full`


## Usage

### Custom Controllers
#### Bringup
Launch the desired controller and Gazebo to try it out:  
``` roslaunch lwr_launch lwr_launch.launch controller:=OneTaskInverseKinematics ```

Use it with a real robot:  
- load the script _lwr_hw/krl/ros_control.src_  and execute it
- ```roslaunch lwr_launch lwr_launch.launch controller:=OneTaskInverseKinematics use_lwr_sim:=false ip:=192.168.0.20 port:=49948 ```  

#### Motion commands
Send commands to the controller in another terminal:

`rostopic pub -1  /lwr/OneTaskInverseKinematics/command_configuration lwr_controllers/PoseRPY '{id: 0, position: {x: -0.4, y: 0.3, z: 1.5}, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

Please refer to the _lwr_controllers_ documentation for further information about the single controllers.

### MoveIt!
__IMPORTANT__:  This is still an EXPERIMENTAL package! Try it out on real hardware at your own risk, and with the red button next to you.
#### Demo
This launch file loads a very simple fake controller that never fails (useful for evaulation of the inverse kinematics):  
`roslaunch lwr_moveit demo.launch`
#### Gazebo Simulation
This launch configuration starts a Gazebo simulation that is controlled by MoveIt!: 
`roslaunch lwr_moveit moveit_planning_execution.launch`
#### Real Robot
This is how MoveIt! can be connected to a real robot:
- load the script _lwr_hw/krl/ros_control.src_  and execute it
- `roslaunch lwr_moveit moveit_planning_execution.launch sim:=false robot_ip:=192.168.0.20 robot_port:=49948`