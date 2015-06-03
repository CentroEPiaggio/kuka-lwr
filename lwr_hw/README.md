# lwr_hw

This package acts much like a driver. It creates a hardware interface to the real/sim robot, and it requires a URDF description in the `robot_description` parameter or configuration file defining the joint names that this hardware interface refers to.

## What to do to run a real LWR 4+

1. Load the script [`lwr_hw/krl/ros_control.src`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_control.src) and the corresponding [`.dat`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_control.src) on the robot.  
2. Place the robot in a position where the joints 1 and 3 are as bent as possible (at least 45 degrees) to avoid the "__FRI interpolation error__". A good way to check this is to go to gravity compensation, and see if the robot enters succesfully in that mode. 
3. Set the robot in __Position__ control.
4. Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button. You can also use the script in semi-automatic mode.  
5. Once the script reaches the loop where it waits for commands, and then, you can proceed to launch your hardware interface + controllers + anything on top of it, e.g. moveit.

### Gravity compensation
(ToDo: revise this procedure after the merge)

In this mode the robot can be moved manually, while the position of each joint is available in TF (__IMPORTANT__: due to the robot own software policies, this mode is only available in T1). This mode can be very useful for calibration, tracking, or teaching.  
1. Load the script [`lwr_hw/krl/ros_monitor.src`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_monitor.src) and the corresponding [`.dat`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_monitor.dat) on the robot.
2. Set the robot in __Position__ control (it will be changed to __Gravity Compensation__ inside the script).
3. Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button (you can also use the script in automatic mode)  
4. The script should reach the loop where it waits for commands
5. Launch the hardware interface.
6. Start the publisher node with 'roslaunch lwr_launch state_publisher.launch'
