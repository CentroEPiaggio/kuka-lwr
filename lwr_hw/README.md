# lwr_hw

This package acts much like a driver. It creates a hardware interface to the real/sim robot, and it requires a URDF description in the `robot_description` parameter or configuration file defining the joint names that this hardware interface refers to.

## What to do to run a real LWR 4+

1. Load the script [`lwr_hw/krl/ros_control.src`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_control.src) and the corresponding [`.dat`](https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_hw/krl/ros_control.src) on the robot. 
2. Place the robot in a position where the joints 1 and 3 are as bent as possible (at least 45 degrees) to avoid the "__FRI interpolation error__". A good way to check this is to go to gravity compensation, and see if the robot enters succesfully in that mode. 
3. Set the robot in __Position__ control.
4. Start the ROS node, it will wait until the robot goes in Monitor mode to avoid loosing UDP packages. ROS controllers will not start until the handshake is done.
5. Start the script with the grey and green buttons; the scripts stops at a point and should be started again by releasing and pressing again the green button. You can also use the script in semi-automatic mode. 
6. From this point on, you can manage/start/stop/run/switch controllers from ROS, and depending on which interface they use, the switch in the KRC unit is done automatically. Everytime there is a switch of interface, it might take a while to get the controllers running again. If the controllers use the same interface, the switch is done only in ROS, which is faster.

### Gravity compensation
(ToDo: revise this procedure after the merge)

In this mode the robot can be moved manually, while the position of each joint is available in TF (__IMPORTANT__: due to the robot own software policies, this mode is only available in T1). This mode can be very useful for calibration, tracking, or teaching.

A hack to avoid stopping everything is to use the joint impedance interface and set all FRI values to zero such that only the gravity term computed internally is sent to the robot. Note that, FRI is command mode during the operation.

To do this, you need to load our [gravity compensation controller](), and then switch to it in ROS as with any other controller.
