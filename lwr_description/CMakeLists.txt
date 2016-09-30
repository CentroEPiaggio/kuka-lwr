cmake_minimum_required(VERSION 2.8.3)
project(lwr_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY model/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/model)
