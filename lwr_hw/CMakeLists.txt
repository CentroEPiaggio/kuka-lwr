cmake_minimum_required(VERSION 2.8.3)
project(lwr_hw)

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
  std_msgs
)

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
    std_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} lwr_hw_gazebo_plugin friremote
  DEPENDS gazebo
)

###########
## Build ##
###########

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## LWR HW BASE CLASS
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/lwr_hw.h
  src/lwr_hw.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## REAL 
# KUKA FRI library
add_library(friremote
  src/fri/friremote.cpp
  src//fri/friudp.cpp
)
# lwr hw fri node
add_executable(lwr_hw_fri_node  src/lwr_hw_fri_node.cpp
  include/${PROJECT_NAME}/lwr_hw_fri.hpp
)
add_dependencies(lwr_hw_fri_node
  friremote
  ${PROJECT_NAME} 
)
target_link_libraries(lwr_hw_fri_node
  friremote
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Standford FRILibrary (support in PROGRESS)
message(WARNING "ATTENTION: This project supports the fork of the Standford library found in https://github.com/iocroblab/fril, and install the libraries in your catkin workspace, not in the system")
include(ExternalProject)
ExternalProject_Add(FRILibrary
  GIT_REPOSITORY https://github.com/iocroblab/fril.git
  GIT_TAG a10695fad32ba8315817c42020e2dbc66f4505dd
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  CONFIGURE_COMMAND cmake -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  INSTALL_COMMAND make install
)
# lwr hw fril node
include_directories(${CATKIN_DEVEL_PREFIX}/include/fril)
link_directories(${CATKIN_DEVEL_PREFIX}/lib)
add_executable(lwr_hw_fril_node  src/lwr_hw_fril_node.cpp
  include/${PROJECT_NAME}/lwr_hw_fril.hpp
)
add_dependencies(lwr_hw_fril_node
  FRILibrary
  ${PROJECT_NAME} 
)
target_link_libraries(lwr_hw_fril_node
  fril
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## SIMULATION
# lwr hw sim gazebo plugin 
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(lwr_hw_gazebo_plugin  src/lwr_hw_gazebo_plugin.cpp
  include/${PROJECT_NAME}/lwr_hw_gazebo.hpp
)
add_dependencies(lwr_hw_gazebo_plugin
  ${PROJECT_NAME}
)
target_link_libraries(lwr_hw_gazebo_plugin
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

## INSTALLATION
install(TARGETS ${PROJECT_NAME} lwr_hw_gazebo_plugin friremote
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS lwr_hw_fri_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/cartesian_hardware_interface/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
