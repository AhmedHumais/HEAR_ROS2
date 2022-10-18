set(HEAR_ROS2_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
set(HEAR_ROS2_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

file(GLOB HEAR_ROS2_SRCs ${HEAR_ROS_SOURCE_DIR}/*.cpp)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)

set(HEAR_ROS2_DEFs "")

if(EXISTS ${TF2_CPP_HEADERS})
  set(HEAR_ROS2_DEFs -DTF2_CPP_HEADERS)
endif()

set(HEAR_ROS2_AMENT_DEPS 
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
)

