cmake_minimum_required(VERSION 2.8.3)
project(mav_linear_mpc)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND NOT APPLE)
  add_definitions(-fPIC)
endif()

#############
# LIBRARIES #
#############
cs_add_library(mav_linear_mpc
  src/linear_mpc.cpp
  src/steady_state_calculation.cpp
)


if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND NOT APPLE)
  target_link_libraries(mav_linear_mpc ${CMAKE_SOURCE_DIR}/lib/libmav_linear_mpc_cvxgen_solver.so)
else()
  target_link_libraries(mav_linear_mpc ${CMAKE_SOURCE_DIR}/lib/libmav_linear_mpc_cvxgen_solver.dylib)
endif()

############
# BINARIES #
############
cs_add_executable(mav_linear_mpc_node
  src/linear_mpc_node.cpp
)
target_link_libraries(mav_linear_mpc_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
