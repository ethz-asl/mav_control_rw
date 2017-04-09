cmake_minimum_required(VERSION 2.8.3)
project(mav_disturbance_observer)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  mav_msgs
  cmake_modules
  roscpp
  tf
  message_generation
  message_runtime
)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/KFDisturbanceObserver.cfg
)

add_message_files(
  FILES
  ObserverState.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES KF_disturbance_observer
  CATKIN_DEPENDS dynamic_reconfigure mav_msgs cmake_modules roscpp tf message_generation message_runtime
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
add_library(KF_disturbance_observer src/KF_disturbance_observer.cpp)
target_link_libraries(KF_disturbance_observer ${catkin_LIBRARIES})
add_dependencies(KF_disturbance_observer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

############
# BINARIES #
############

#No binaries, only libraries in this package!

##########
# EXPORT #
##########
install(TARGETS KF_disturbance_observer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
