cmake_minimum_required(VERSION 3.8)
project(lio_evaluation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lio_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree filters)
find_package(Eigen3 REQUIRED)

# ---- External Geometry Proxy Detector (M3) ----
add_executable(degeneracy_proxy_node src/degeneracy_proxy.cpp)
target_include_directories(degeneracy_proxy_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(degeneracy_proxy_node ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(degeneracy_proxy_node
  rclcpp sensor_msgs nav_msgs std_msgs lio_interfaces pcl_conversions)

install(TARGETS degeneracy_proxy_node
  DESTINATION lib/${PROJECT_NAME})

# ---- Python 工具 (M4) ----
install(PROGRAMS scripts/recorder_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME recorder_node)
install(PROGRAMS scripts/monitor_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME monitor_node)
install(PROGRAMS scripts/drive_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME drive_node)
install(PROGRAMS scripts/health_watch.py
  DESTINATION lib/${PROJECT_NAME} RENAME health_watch)
install(PROGRAMS scripts/compute_metrics.py
  DESTINATION lib/${PROJECT_NAME} RENAME compute_metrics)
install(PROGRAMS scripts/run_experiment.py
  DESTINATION lib/${PROJECT_NAME} RENAME run_experiment)
install(PROGRAMS scripts/directional_analysis.py
  DESTINATION lib/${PROJECT_NAME} RENAME directional_analysis)
install(PROGRAMS scripts/generate_worlds.py
  DESTINATION lib/${PROJECT_NAME} RENAME generate_worlds)
# ---- P2 误差归因 (attribution + fault injector) ----
install(PROGRAMS scripts/attribution_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME attribution_node)
install(PROGRAMS scripts/attribution_rules.py
  DESTINATION lib/${PROJECT_NAME})   # 保留 .py 后缀: attribution_node 运行时 import
install(PROGRAMS scripts/attribution_validation.py
  DESTINATION lib/${PROJECT_NAME} RENAME attribution_validation)
install(PROGRAMS scripts/lidar_fault_injector.py
  DESTINATION lib/${PROJECT_NAME} RENAME lidar_fault_injector)
install(PROGRAMS scripts/parity_check.py
  DESTINATION lib/${PROJECT_NAME} RENAME parity_check)
install(PROGRAMS scripts/p3_ab_compare.py
  DESTINATION lib/${PROJECT_NAME} RENAME p3_ab_compare)
# ---- P5 (P5.0 first-iteration 分布分析; 其余 p5_*.py 走 src 直跑, 同 P4 惯例) ----
install(PROGRAMS scripts/p5_analyze_first_iter.py
  DESTINATION lib/${PROJECT_NAME})

# ---- launch / config ----
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
