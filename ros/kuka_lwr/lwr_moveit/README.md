# KUKA LWR

ROS indigo metapackage that contains ROS related code to work with the KUKA LWR 4+

## Dependencies

For control and simulation (_lwr_controllers_):

`sudo apt-get install gazebo ros-indigo-gazebo-ros-control ros-indigo-ros-control ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-joint-trajectory-controller ros-indigo-joint-trajectory-action`

For planning (_lwr_moveit_):

`sudo apt-get install ros-indigo-moveit-full`


## Usage

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