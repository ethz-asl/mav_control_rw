cmake_minimum_required(VERSION 2.8.3)
project(mav_lowlevel_attitude_controller)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  mav_msgs
  std_msgs
  cmake_modules
  roscpp
  dynamic_reconfigure
  tf
)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PIDAttitude.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES pid_attitude_controller
  CATKIN_DEPENDS mav_msgs std_msgs cmake_modules roscpp dynamic_reconfigure tf
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
add_library(pid_attitude_controller src/PID_attitude_controller.cc)

target_link_libraries(pid_attitude_controller ${catkin_LIBRARIES})
add_dependencies(pid_attitude_controller ${catkin_EXPORTED_TARGETS})

############
# BINARIES #
############
add_executable(mav_pid_attitude_controller_node src/PID_attitude_controller_node.cc)
target_link_libraries(mav_pid_attitude_controller_node pid_attitude_controller ${catkin_LIBRARIES})
add_dependencies(mav_pid_attitude_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mav_pid_attitude_controller_node ${PROJECT_NAME}_gencfg)


##########
# EXPORT #
##########
install(TARGETS pid_attitude_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS mav_pid_attitude_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
