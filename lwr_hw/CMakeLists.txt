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
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)

add_definitions (-fpermissive)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_manager
    pluginlib
    kdl_parser
#  INCLUDE_DIRS include
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

## LWR HW BASE CLASS
add_library(lwr_hw
  src/lwr_hw.cpp
)

## REAL 
## lwr hw real library
add_library(friremote
  src/fri/friremote.cpp
  src//fri/friudp.cpp
)
add_library(lwr_hw_real
  src/lwr_hw_real.cpp
)
add_dependencies(lwr_hw_real
  lwr_hw
  friremote
)
target_link_libraries(lwr_hw_real
  lwr_hw
  friremote
  ${catkin_LIBRARIES}
)

## lwr hw real node
add_executable(lwr_hw_node
  src/lwr_hw_real_node.cpp
)
target_link_libraries(lwr_hw_node
  lwr_hw_real
  ${catkin_LIBRARIES}
)

## SIMULATION

## lwr hw sim base class
# add_library(lwr_hw_sim
#  src/lwr_hw_sim.cpp
#)

## lwr hw sim default behavior
add_library(default_lwr_hw_sim 
  src/default_lwr_hw_sim.cpp
)
# add_dependencies(default_lwr_hw_sim
#  lwr_hw_sim
#)
target_link_libraries(default_lwr_hw_sim
#  lwr_hw_sim
  lwr_hw # this is needed because there is no lwr_hw_sim library
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

# lwr hw sim gazebo plugin
add_library(lwr_hw_sim_plugin
  src/lwr_hw_sim_plugin.cpp
)
target_link_libraries(lwr_hw_sim_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

## Install
install(TARGETS default_lwr_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES lwr_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )