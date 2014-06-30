cmake_minimum_required(VERSION 2.8.3)
project(kuka_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  controller_interface
  dynamic_reconfigure
  forward_command_controller
  realtime_tools
  urdf
  Eigen
)

# catkin_package(
#    CATKIN_DEPENDS  roscpp angles filters tf tf_conversions  eigen_conversions pluginlib actionlib
#     std_msgs geometry_msgs diagnostic_msgs trajectory_msgs pr2_controllers_msgs 
#     pr2_controller_interface pr2_controller_manager kdl_parser pr2_mechanism_model control_toolbox realtime_tools message_filters control_msgs 
#    INCLUDE_DIRS include
#    LIBRARIES robot_mechanism_controllers
# )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    eigen_conversions
    geometry_msgs
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )


add_library(${PROJECT_NAME}
  src/joint_impedance_controller.cpp
  src/kuka_joint_effort_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES kuka_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
