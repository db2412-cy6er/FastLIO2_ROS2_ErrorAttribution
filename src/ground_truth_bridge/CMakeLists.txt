cmake_minimum_required(VERSION 3.10)
project(ground_truth_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ground_truth_bridge_node src/ground_truth_bridge.cpp)
target_include_directories(ground_truth_bridge_node PRIVATE include)
ament_target_dependencies(ground_truth_bridge_node
  rclcpp nav_msgs geometry_msgs)

install(TARGETS ground_truth_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()