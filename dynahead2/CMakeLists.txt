cmake_minimum_required(VERSION 2.8.3)
project(dynahead2)

#==
# Dependencies
#==
find_package(catkin)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)