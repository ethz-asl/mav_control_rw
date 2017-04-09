cmake_minimum_required(VERSION 2.8.3)
project(mav_nonlinear_mpc)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

# On os x we need to use openblas for the lapacke library
find_package(OpenBLAS REQUIRED)
if (Openblas_FOUND)
  message("-- Found Openblas library: ${Openblas_LIBRARIES}")
  message("-- Found Openblas include dirs: ${Openblas_INCLUDE_DIRS}")
  include_directories(${Openblas_INCLUDE_DIRS})
  set(lapacke_LIBRARIES ${Openblas_LIBRARIES})
else (Openblas_FOUND)
  message("-- Openblas not found, trying to use lapacke")
  find_package(LAPACK REQUIRED)
  set(lapacke_LIBRARIES lapacke)
endif (Openblas_FOUND)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	mav_msgs
	dynamic_reconfigure
	tf
	mav_control_interface
	mav_disturbance_observer
)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/NonLinearMPC.cfg
  )

catkin_package(
	INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
	LIBRARIES mav_nonlinear_mpc
	CATKIN_DEPENDS cmake_modules roscpp mav_msgs dynamic_reconfigure tf mav_control_interface mav_disturbance_observer
	DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

if(APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()





#############
# LIBRARIES #
#############
add_library(mav_nonlinear_mpc
  src/nonlinear_mpc.cc
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/acado_qpoases_interface.cpp
  solver/acado_integrator.c
  solver/acado_solver.c
  solver/acado_auxiliary_functions.c
)

target_include_directories(mav_nonlinear_mpc PUBLIC
	solver/
	solver/qpoases
	solver/qpoases/INCLUDE
	solver/qpoases/SRC
)

target_link_libraries(mav_nonlinear_mpc ${lapacke_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(mav_nonlinear_mpc ${catkin_EXPORTED_TARGETS})

############
# BINARIES #
############
add_executable(nonlinear_mpc_node src/nonlinear_mpc_node.cc)
target_link_libraries(nonlinear_mpc_node mav_nonlinear_mpc ${catkin_LIBRARIES})
add_dependencies(nonlinear_mpc_node ${catkin_EXPORTED_TARGETS})
add_dependencies(nonlinear_mpc_node ${PROJECT_NAME}_gencfg)

##########
# EXPORT #
##########
install(TARGETS mav_nonlinear_mpc
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS nonlinear_mpc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
