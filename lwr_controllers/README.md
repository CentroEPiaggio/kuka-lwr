# KUKA LWR CONTROLLERS

ROS/indigo package with various controllers implemented for the LWR 4+

## Overview

This package contains the implementation of different control strategies for the KUKA LWR 4+. 


## Usage

The desired controller can be ran with the aid of the launch files from the _lwr_launch_ package. 
The following example loads the OneTaskInverseKinematics controller:  
```roslaunch lwr_launch lwr_launch.launch controller:=OneTaskInverseKinematics```  
Each of the controllers listed here can be loaded this way, either for use on a simulated robot or on a real one, depending on the value of the _use_lwr_sim_ parameter.  

Also more than one controller can be loaded at once:  
```roslaunch lwr_launch lwr_launch.launch controller:="joint_position_controller joint_trajectory_controller"```  
(please note the quotes surrounding the list of controllers to be loaded).  

Sometimes it is desired to load multiple controllers, but start only one of them, with the aim of switch between them at runtime with controller_manager.  
This can be achieved with the _stopped_controllers_ parameter:  
```roslaunch lwr_launch lwr_launch.launch controller:="OneTaskInverseKinematics"  stopped_controllers:="ComputedTorqueControl"```  

Once the planning environment has loaded, commands can be sent to any joint in another terminal.

- Impedance Controller:
`rostopic pub -1  /lwr/JointImpedanceControl/command std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- Inverse Dynamics Controller
`rostopic pub -1  /lwr/InverseDynamicsControl/ext_wrench geometry_msgs/WrenchStamped '{wrench: {force:  {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0,y: 0.0,z: 0.0}}}' `

- Computed Torque Controller:
`rostopic pub -1  /lwr/ComputedTorqueControl/command std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- One Task Inverse Kinematics Controller: 

  - Full pose (note the IDs)
`rostopic pub -1  /lwr/OneTaskInverseKinematics/command lwr_controllers/PoseRPY '{id: 0, position: {x: -0.4, y: 0.3, z: 1.5}, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

  - Position only
`rostopic pub -1  /lwr/OneTaskInverseKinematics/command lwr_controllers/PoseRPY '{id: 1, position: {x: -0.4, y: 0.3, z: 1.5}}'`

  - Orientation only
`rostopic pub -1  /lwr/OneTaskInverseKinematics/command lwr_controllers/PoseRPY '{id: 2, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

  - PID Gains setting
`rosrun rqt_reconfigure rqt_reconfigure`

- Multi Task Priority Inverse Kinematics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`rostopic pub -1  /lwr/MultiTaskPriorityInverseKinematics/command lwr_controllers/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

  - PID Gains setting
`rosrun rqt_reconfigure rqt_reconfigure`

- Multi Task Priority Inverse Dynamics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`rostopic pub -1  /lwr/MultiTaskPriorityInverseDynamics/command lwr_controllers/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

- Dynamic Sliding PID Control (Task Space)
`rostopic pub -1 /lwr/DynamicSlidingModeControllerTaskSpace/command std_msgs/Float64MultiArray '{data:[-0.4,0.3,1.5,0,0,0]}'`

- Minimum Effort Inverse Dynamics Controller:
just run the controller (no command support)

- Backstepping Controller:
just run the controller (no command support)






