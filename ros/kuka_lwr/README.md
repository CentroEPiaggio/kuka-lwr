KUKA LWR
========

ROS/hydro metapackage that contains ROS related code to work with the KUKA LWR 4+

Overview
--------


Build
-----

* Dependencies

For control and simulation (lwr_control, lwr_gazebo, )

`sudo apt-get install gazebo ros-hydro-gazebo-ros-control ros-hydro-ros-control ros-hydro-effort-controllers ros-hydro-joint-state-controller ros-hydro-joint-trajectory-controller ros-hydro-joint-trajectory-action`

For planning:

`sudo apt-get install ros-hydro-moveit-full`


Use
---

* Gazebo-based simulator

Launch the simulation environment in terminal 1:

`roslaunch lwr_gazebo lwr_on_box.launch`

Launch the controllers in terminal 2:

`roslaunch lwr_ros_control lwr_control.launch`

Send commands to any joint in terminal 3:

`rostopic pub -1 /lwr/joint0_position_controller/command std_msgs/Float64 "data: 0.0"`

`rostopic pub -1 /lwr/joint1_position_controller/command std_msgs/Float64 "data: 0.0"`

`rostopic pub -1 /lwr/joint2_position_controller/command std_msgs/Float64 "data: 0.0"`

`rostopic pub -1 /lwr/joint3_position_controller/command std_msgs/Float64 "data: 0.0"`

`rostopic pub -1 /lwr/joint4_position_controller/command std_msgs/Float64 "data: 0.0"`

`rostopic pub -1 /lwr/joint5_position_controller/command std_msgs/Float64 "data: 0.0"`

`rostopic pub -1 /lwr/joint6_position_controller/command std_msgs/Float64 "data: 0.0"`

