cmake_minimum_required(VERSION 2.8.3)
project(mav_control_interface)

find_package(catkin REQUIRED)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  mav_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  visualization_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES mav_control_interface
	CATKIN_DEPENDS cmake_modules roscpp mav_msgs nav_msgs std_msgs std_srvs tf tf_conversions visualization_msgs
	DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

set(SOURCE_FILES
  src/mav_control_interface.cpp
  src/mav_control_interface_impl.cpp
  src/parameters.cpp
  src/position_controller_interface.cpp
  src/rc_interface.cpp
  src/rc_interface_aci.cpp
  src/state_machine.cpp
  src/mpc_queue.cpp
)

#############
# LIBRARIES #
#############
add_library(mav_control_interface ${SOURCE_FILES})
SET_TARGET_PROPERTIES(mav_control_interface PROPERTIES COMPILE_FLAGS "-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=30 -DBOOST_MPL_LIMIT_MAP_SIZE=30 ")

target_link_libraries(mav_control_interface ${catkin_LIBRARIES})
add_dependencies(mav_control_interface ${catkin_EXPORTED_TARGETS})

############
# BINARIES #
############
add_executable(mav_control_interface_test_node src/test_node.cpp)
target_link_libraries(mav_control_interface_test_node mav_control_interface ${catkin_LIBRARIES})
add_dependencies(mav_control_interface_test_node ${catkin_EXPORTED_TARGETS})

##########
# EXPORT #
##########
install(TARGETS mav_control_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS mav_control_interface_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
