std_msgs/Header header
# 1. If frame_id in header is NOT EMPTY, then x_FRI is considered to be the pose w.r.t. that frame, so the increment w.r.t. the current tip pose will be computed internally (in the teach pendant is equivalent to select the Cartesian increments to be in the WORLD frame) 
# 2. If frame_id in header is EMPTY, then x_FRI is considered to be the pose w.r.t. to the current tip pose (in the teach pendant is equivalent to select the Cartesian increments to be in the TOOL frame)
# The increment (or offset) is x_FRI
geometry_msgs/Pose x_FRI
# The stiffness, damping and wrench are expressed in the current tip frame readed in the HWiface
lwr_controllers/Stiffness k_FRI
lwr_controllers/Stiffness d_FRI
geometry_msgs/Wrench f_FRI