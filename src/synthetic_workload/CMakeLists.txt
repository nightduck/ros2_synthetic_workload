cmake_minimum_required(VERSION 3.5)
project(synthetic_workload)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(mt_exec_shared_node src/mt_exec_shared_node.cpp)
ament_target_dependencies(mt_exec_shared_node rclcpp std_msgs)

add_executable(mt_exec_personal_nodes src/mt_exec_personal_nodes.cpp)
ament_target_dependencies(mt_exec_personal_nodes rclcpp std_msgs)

install(TARGETS
  mt_exec_shared_node mt_exec_personal_nodes
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()