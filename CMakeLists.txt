cmake_minimum_required(VERSION 3.5)
project(turtle_catch)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

add_executable(turtle_spawn_node nodes/turtle_spawn_node.cpp src/turtle_spawn.cpp)
ament_target_dependencies(turtle_spawn_node rclcpp turtlesim)

install(TARGETS
  turtle_spawn_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
