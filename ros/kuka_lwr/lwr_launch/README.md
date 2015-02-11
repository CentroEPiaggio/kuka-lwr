 
# lwr_launch

This package contains the launch files needed to use the controllers contained in lwr_controllers, together with the simulated robot in Gazebo or with the real robot via lwr_hw.  
MoveIt! related file are excluded, as they are located in the lwr_moveit/launch directory.  

## Usage

### Gravity compensation
In this mode the robot can be moved manually, while the position of each joint is available in TF (__IMPORTANT__: due to the robot's own 
software policies, this mode is only available in T1). This mode can be very useful for calibration or tracking procedures.  
- load the script _lwr_hw/krl/ros_monitor.src_ on the robot  
- put the robot in __Position__ control (it will be changed to __Gravity Compensation__ inside the script)  
- start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button (you can also use the script in automatic mode)  
- the script should reach the loop where it waits for commands
- start the publisher node with 'roslaunch lwr_launch state_publisher.launch'

### Simulator 
_lwr_launch.launch_ starts by default the OneTaskInverseKinematics controller and the robot simulator in Gazebo.  
The robot can be sent to an arbitrary pose by publishing a message of type __lwr_controllers/PoseRPY__ on the _/lwr/OneTaskInverseKinematics/command_configuration_ topic:

    rostopic pub -1 /lwr/OneTaskInverseKinematics/command_configuration lwr_controllers/PoseRPY '{id: 0, position: {x: -0.4, y: 0.3, z: 1.5}, orientation: {roll: 0.1, pitch: 0.2, yaw: 0.0}}'

Please refer to the _lwr_controllers_ documentation for further information about available controllers.  

### Connection to the real robot
Setting the __use_lwr_sim__ argument to _false_ replaces the Gazebo simulation with a connection to the real robot through the _lwr_hw_ package. IP and port of the FRI connection can be set through the __IP__ and __PORT__ arguments.
