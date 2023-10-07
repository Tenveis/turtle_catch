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
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Turtle.msg"
  "msg/TurtleArray.msg"
)

# Generate relevant files of custom interface
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)


#include '.h' files
include_directories(include)

add_executable(turtle_spawn_node nodes/turtle_spawn_node.cpp src/turtle_spawn.cpp)
ament_target_dependencies(turtle_spawn_node rclcpp turtlesim)

# reference from here: https://robotics.stackexchange.com/questions/23171/creating-a-custom-msg-in-ros2
# include custom interface in the same package.
# for ros-galactic and below
rosidl_target_interfaces(turtle_spawn_node ${PROJECT_NAME} "rosidl_typesupport_cpp") 

## for ros-humble and above, >>>>>comment above line and uncomment below two line.
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(turtle_spawn_node "${cpp_typesupport_target}") 

install(TARGETS
  turtle_spawn_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
