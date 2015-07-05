cmake_minimum_required(VERSION 2.8.3)
project(lwr_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  roscpp
  tf
  urdf
  cmake_modules
  pluginlib
  kdl_parser
  transmission_interface
  gazebo_ros
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)

add_definitions (-fpermissive -std=c++11)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    joint_limits_interface
    roscpp
    tf
    urdf
    cmake_modules
    pluginlib
    kdl_parser
    transmission_interface
    gazebo_ros
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} lwr_hw_sim_plugin friremote
  DEPENDS gazebo
)

###########
## Build ##
###########

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

## LWR HW BASE CLASS TO RULE'EM ALL
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/lwr_hw.h
  src/lwr_hw.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## REAL 
## FRI library
add_library(friremote
  src/fri/friremote.cpp
  src//fri/friudp.cpp
)
## lwr hw real node
add_executable(lwr_hw_node  src/lwr_hw_real_node.cpp
  include/${PROJECT_NAME}/lwr_hw_real.hpp
)
add_dependencies(lwr_hw_node
  friremote
  ${PROJECT_NAME} 
)
target_link_libraries(lwr_hw_node
  friremote
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## SIMULATION
# lwr hw sim gazebo plugin
add_library(lwr_hw_sim_plugin  src/lwr_hw_sim_plugin.cpp
  include/${PROJECT_NAME}/lwr_hw_sim.hpp
)
add_dependencies(lwr_hw_sim_plugin
  ${PROJECT_NAME}
)
target_link_libraries(lwr_hw_sim_plugin
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
