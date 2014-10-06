KUKA LWR
========

ROS/hydro metapackage that contains ROS related code to emulate the controllers included in the KUKA LWR 4+

Overview
--------


Build
-----

* 

Launch the simulation environment in terminal 1:

`roslaunch lwr_gazebo lwr_on_box.launch`

Launch the controllers in terminal 2:

`roslaunch lwr_ros_control `

You have to be sure that in lwr_control.launch is active the controller that you are looking for

Send commands to any joint in terminal 3 (you need to load/start the position controllers in the lwr_control.launch file):

- Impedance Controller:
`rostopic pub -1  /JointImpedanceControl/command_configuration std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- Inverse Dynamics Controller
`rostopic pub -1  /InverseDynamicsControl/ext_wrench geometry_msgs/WrenchStamped '{wrench: {force:  {x: 0.0, y: 0.0, z: 0.0}, torque: {x: 0.0,y: 0.0,z: 0.0}}}' `

- Computed Torque Controller:
`rostopic pub -1  /ComputedTorqueControl/command_configuration std_msgs/Float64MultiArray '{ data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'`

- One Task Inverse Kinematics Controller: 

  - Full pose (note the IDs)
`rostopic pub -1  /OneTaskInverseKinematics/command_configuration kuka_controllers/PoseRPY '{id: 0, position: {x: -0.4, y: 0.3, z: 1.5}, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

  - Position only
`rostopic pub -1  /OneTaskInverseKinematics/command_configuration kuka_controllers/PoseRPY '{id: 1, position: {x: -0.4, y: 0.3, z: 1.5}}'`

  - Orientation only
`rostopic pub -1  /OneTaskInverseKinematics/command_configuration kuka_controllers/PoseRPY '{id: 2, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'`

  - PID Gains setting
`rostopic pub -1  /OneTaskInverseKinematics/set_gains std_msgs/Float64MultiArray '{data: [60, 1.2, 10]}'`

- Multi Task Priority Inverse Kinematics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`rostopic pub -1  /MultiTaskPriorityInverseKinematics/command_configuration kuka_controllers/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

  - PID Gains setting
`rostopic pub -1  /MultiTaskPriorityInverseKinematics/set_gains std_msgs/Float64MultiArray '{data: [60, 1.2, 10]}'`

- Multi Task Priority Inverse Dynamics Controller:
  
  - Tasks description (Notes: Link index -1 is EndEffector and Task parameters are [x,y,x,roll,pitch,yaw])
`rostopic pub -1  /MultiTaskPriorityInverseDynamics/command_configuration kuka_controllers/MultiPriorityTask '{links: [-1,3], tasks: [-0.4,0.3,1.5,0,0,0,-0.02,0.2,1.311,-1.568,0.291,0.1]}'`

- Minimum Effort Inverse Dynamics Controller:
just run the controller (no command support)

- Backstepping Controller:
just run the controller (no command support)






