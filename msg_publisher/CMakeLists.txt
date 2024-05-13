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
find_package(ament_cmake_auto REQUIRED)
find_package(vm_msgs REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_executable(${PROJECT_NAME}_node
  src/node.cpp
  src/DbwNode.cpp
)

ament_auto_add_executable(can2ros_node
  src/can2node.cpp
  src/CAN2ROS.cpp
)

ament_target_dependencies(msg_publisher_node vm_msgs)
ament_target_dependencies(can2ros_node vm_msgs)

target_compile_options(${PROJECT_NAME}_node PRIVATE -Wno-unused-function)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
