cmake_minimum_required(VERSION 3.5.1)
project(dynahead2_vr)

# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(PACKAGE_DEPENDENCIES
    any_node
    sensor_msgs
    dynahead2_control
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
  src/Dynahead2VR.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_executable(dynahead2_vr_node
  src/dynahead2_vr_node.cpp
)

add_dependencies(dynahead2_vr_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dynahead2_vr_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} dynahead2_vr_node
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
