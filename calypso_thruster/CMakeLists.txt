cmake_minimum_required(VERSION 3.0.2)
project(calypso_thruster)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  visualization_msgs
  sensor_msgs
  gazebo_dev
  tf2_ros
  tf2
)

find_package(catkin REQUIRED COMPONENTS gazebo_dev)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/msgs)
file(GLOB msgs msgs/*.proto)

################################################################
##################### sets #####################################
################################################################

set(CALYPSO_GAZEBO_PLUGINS_LIST "")


set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  FloatStamped.msg
  ThrusterConversionFcn.msg
)

add_service_files(
  FILES
  SetThrusterEfficiency.srv
  SetThrusterState.srv
  GetThrusterEfficiency.srv
  GetThrusterState.srv
  GetThrusterConversionFcn.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include 
    ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
    ${GAZEBO_MSG_INCLUDE_DIRS} 
  LIBRARIES
    calypso_thruster_plugin
    calypso_dynamics
    calypso_thruster_ros_plugin
  CATKIN_DEPENDS
    sensor_msgs
    std_msgs
    geometry_msgs
    message_runtime
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

###########
## Build ##
###########

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} # for generated messages
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(calypso_gazebo_plugins_msgs SHARED ${PROTO_SRCS})

add_library(calypso_dynamics
    SHARED
    src/Dynamics.cc 
)

add_library(calypso_thruster_ros_plugin src/ThrusterROSPlugin.cc)


add_library(calypso_thruster_plugin
    src/ThrusterConversionFcn.cc
    src/ThrusterPlugin.cc
)

add_dependencies(calypso_thruster_ros_plugin ${catkin_EXPORTED_TARGETS})
add_dependencies(calypso_thruster_plugin calypso_gazebo_plugins_msgs ${catkin_EXPORTED_TARGETS})

target_link_libraries(calypso_gazebo_plugins_msgs ${PROTOBUF_LIBRARY})
target_link_libraries(calypso_thruster_ros_plugin ${catkin_LIBRARIES})
target_link_libraries(calypso_thruster_plugin
    calypso_dynamics
    calypso_gazebo_plugins_msgs
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

####################################################################
################# APPEND LISTS #####################################
####################################################################

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})

list(APPEND CALYPSO_GAZEBO_PLUGINS_LIST calypso_gazebo_plugins_msgs)

list(APPEND CALYPSO_GAZEBO_PLUGINS_LIST calypso_dynamics)

list(APPEND CALYPSO_GAZEBO_ROS_PLUGINS_LIST calypso_thruster_ros_plugin)

list(APPEND CALYPSO_GAZEBO_PLUGINS_LIST calypso_thruster_plugin)

#########################################################################
##################### INSTALL TARGETS ###################################
#########################################################################

install(TARGETS ${CALYPSO_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${CALYPSO_GAZEBO_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.pb.*"
  PATTERN "*~" EXCLUDE
)