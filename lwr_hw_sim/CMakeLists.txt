cmake_minimum_required(VERSION 2.8.3)
project(lwr_hw_sim)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  gazebo_ros_control
  kdl_parser
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_manager
    pluginlib
    transmission_interface
    gazebo_ros_control
    kdl_parser
#  INCLUDE_DIRS include
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Libraries
add_library(default_lwr_hw_sim src/default_lwr_hw_sim.cpp)
target_link_libraries(default_lwr_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Install
install(TARGETS default_lwr_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  )

install(FILES lwr_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )