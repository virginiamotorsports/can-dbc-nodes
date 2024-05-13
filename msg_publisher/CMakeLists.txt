cmake_minimum_required(VERSION 3.5)
project(msg_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vm_msgs REQUIRED)
find_package(can_dbc_parser REQUIRED)

include_directories(include)
add_library(raptor_lib SHARED
  src/DbwNode.cpp
)

ament_target_dependencies(raptor_lib
  rclcpp
  rclcpp_components
  vm_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  can_dbc_parser
)

rclcpp_components_register_node(raptor_lib
  PLUGIN "msg_publisher::DbwNode"
  EXECUTABLE can_frame_to_msg_node
)

install(TARGETS raptor_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
