CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
PROJECT(nmpc_solver)

# CMake module(s) path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set (CMAKE_CXX_FLAGS  "-std=c++11 -g")
#
# Prerequisites
#
FIND_PACKAGE(ACADO REQUIRED)

#
# Include directories
#
INCLUDE_DIRECTORIES( ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
ADD_EXECUTABLE(../solver/nmpc_solver_setup nmpc_solver_setup.cpp )

TARGET_LINK_LIBRARIES(../solver/nmpc_solver_setup 
  ${ACADO_SHARED_LIBRARIES}
)

SET_TARGET_PROPERTIES(../solver/nmpc_solver_setup PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

