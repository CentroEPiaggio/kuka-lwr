cmake_minimum_required(VERSION 2.8.3)
project(lwr_launch)

find_package(catkin REQUIRED)

catkin_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  launch/lwr_launch.launch
  launch/rviz_config.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
