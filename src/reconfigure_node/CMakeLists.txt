cmake_minimum_required(VERSION 3.5)

project(reconfigure_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(node_test src/node_test.cpp)
ament_target_dependencies(node_test rclcpp std_msgs example_interfaces)

add_executable(node_alex src/alex_example.cpp)
ament_target_dependencies(node_alex rclcpp std_msgs example_interfaces)

add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(simple_node rclcpp std_msgs example_interfaces)

add_executable(simple_node_declared src/simple_node_declared.cpp)
ament_target_dependencies(simple_node_declared rclcpp std_msgs example_interfaces)

install(
    TARGETS node_test node_alex simple_node simple_node_declared
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()