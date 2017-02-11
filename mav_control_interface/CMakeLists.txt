cmake_minimum_required(VERSION 2.8.3)
project(mav_control_interface)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

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

cs_add_library(${PROJECT_NAME} ${SOURCE_FILES})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=30 -DBOOST_MPL_LIMIT_MAP_SIZE=30 ")


cs_add_executable(${PROJECT_NAME}_test_node src/test_node.cpp)
target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME})

cs_install()
cs_export()
