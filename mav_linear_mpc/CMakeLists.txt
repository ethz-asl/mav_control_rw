cmake_minimum_required(VERSION 2.8.3)
project(mav_linear_mpc)

set(CMAKE_BUILD_TYPE Release)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND NOT APPLE)
  add_definitions(-fPIC)
endif()

find_package(catkin REQUIRED COMPONENTS
  mav_msgs
  roscpp
  cmake_modules
  tf
  mav_control_interface
  mav_disturbance_observer
  dynamic_reconfigure
)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LinearMPC.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES mav_linear_mpc
  CATKIN_DEPENDS mav_msgs roscpp cmake_modules tf mav_control_interface mav_disturbance_observer dynamic_reconfigure
  DEPENDS Eigen
)

include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
add_library(mav_linear_mpc
  src/linear_mpc.cpp
  src/steady_state_calculation.cpp
)

add_library(cvxgen_solver
  lib/ldl.c
  lib/solver.c
  lib/matrix_support.c
  lib/util.c
  lib/variables_definition.c
)

target_link_libraries(mav_linear_mpc cvxgen_solver ${catkin_LIBRARIES})
add_dependencies(mav_linear_mpc ${catkin_EXPORTED_TARGETS})


############
# BINARIES #
############
add_executable(mav_linear_mpc_node src/linear_mpc_node.cpp)
target_link_libraries(mav_linear_mpc_node mav_linear_mpc ${catkin_LIBRARIES})
add_dependencies(mav_linear_mpc_node ${catkin_EXPORTED_TARGETS})
add_dependencies(mav_linear_mpc_node ${PROJECT_NAME}_gencfg)

##########
# EXPORT #
##########
install(TARGETS mav_linear_mpc
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS mav_linear_mpc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
