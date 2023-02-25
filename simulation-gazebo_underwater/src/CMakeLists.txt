find_package(Boost 1.40 COMPONENTS system REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

if (TARGET Boost::system)
    list(APPEND __boost_deps DEPS_TARGET Boost::system)
else()
    list(APPEND __boost_deps DEPS_PLAIN Boost)
endif()

foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".*gazebo-[0-9]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

protobuf_generate_cpp(uw_msgs_SRCS uw_msgs_HDR uw_msgs.proto
    ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
    ${PROTOBUF_IMPORT_DIRS}/header.proto
    ${PROTOBUF_IMPORT_DIRS}/time.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

rock_library(gazebo_underwater_msgs
    SOURCES  ${uw_msgs_SRCS}
    HEADERS ${uw_msgs_HDR} DataTypes.hpp Gazebo7Shims.hpp
    DEPS_PKGCONFIG gazebo protobuf)

rock_library(gazebo_underwater
    SOURCES GazeboUnderwater.cpp
    HEADERS GazeboUnderwater.hpp
    ${__boost_deps}
    DEPS_PKGCONFIG gazebo
    DEPS gazebo_underwater_msgs)

#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/msgs.pb.h
#    DESTINATION include/${PROJECT_NAME})

install(FILES uw_msgs.proto
    DESTINATION include/${PROJECT_NAME}/proto)
