cmake_minimum_required(VERSION 2.8.3)
project(mav_lowlevel_attitude_controller)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

#############
# LIBRARIES #
#############
cs_add_library(pid_attitude_controller_lib
  src/PID_attitude_controller.cc
  )

############
# BINARIES #
############
cs_add_executable(mav_pid_attitude_controller_node
  src/PID_attitude_controller_node.cc
)

target_link_libraries(mav_pid_attitude_controller_node pid_attitude_controller_lib)

add_dependencies(mav_pid_attitude_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS})


##########
# EXPORT #
##########
cs_install()
cs_export()
