cmake_minimum_required(VERSION 2.8.3)
project(platinum_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

# set(CMAKE_PREFIX_PATH "build/roxanne_rosjava_msgs/cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  actionlib
  roxanne_rosjava_msgs
  cmake_modules
  message_generation
  geometry_msgs
)

find_package(TinyXML REQUIRED)
# find_package( REQUIRED)

add_service_files(
	DIRECTORY
	srv
)

generate_messages(DEPENDENCIES std_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES platinum_bridge ${EXTERNAL_LIBS}
  CATKIN_DEPENDS roscpp rospy roxanne_rosjava_msgs actionlib geometry_msgs
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/bridge.cpp
 )

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(bridge src/bridge.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bridge
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${EXTERNAL_LIBS}
  ${TinyXML_LIBRARIES}
 )

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
