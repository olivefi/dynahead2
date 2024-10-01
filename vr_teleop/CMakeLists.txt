cmake_minimum_required(VERSION 3.5.1)
project(vr_teleop)

# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(PACKAGE_DEPENDENCIES
    any_node
    sensor_msgs
    geometry_msgs
)

find_package(catkin
    REQUIRED COMPONENTS
      ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
#  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/VRTeleop.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_executable(vr_teleop_node
  src/vr_teleop_node.cpp
)

add_dependencies(vr_teleop_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(vr_teleop_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} vr_teleop_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch"
)
