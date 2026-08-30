cmake_minimum_required(VERSION 3.16)
project(global_relocalization_kiss_matcher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# Ubuntu 22.04's libflann-dev does not ship FLANNConfig.cmake.
find_path(FLANN_INCLUDE_DIR NAMES flann/flann.hpp REQUIRED)
find_library(FLANN_CPP_LIBRARY NAMES flann_cpp REQUIRED)

# Build the pinned ROBIN dependency entirely from local source. Its upstream
# CMake downloads PMC and xenium unconditionally, so define only the two
# libraries needed by KISS-Matcher and skip all tools, tests and installers.
set(THIRD_PARTY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party")
set(PMC_ROOT "${THIRD_PARTY_ROOT}/pmc")
add_library(pmc_core STATIC
  ${PMC_ROOT}/pmc_heu.cpp
  ${PMC_ROOT}/pmc_maxclique.cpp
  ${PMC_ROOT}/pmcx_maxclique.cpp
  ${PMC_ROOT}/pmcx_maxclique_basic.cpp
  ${PMC_ROOT}/pmc_cores.cpp
  ${PMC_ROOT}/pmc_utils.cpp
  ${PMC_ROOT}/pmc_graph.cpp
  ${PMC_ROOT}/pmc_clique_utils.cpp
)
set_target_properties(pmc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pmc_core PUBLIC ${PMC_ROOT}/include)
target_link_libraries(pmc_core PUBLIC OpenMP::OpenMP_CXX)

set(ROBIN_ROOT "${THIRD_PARTY_ROOT}/robin")
add_library(robin_core STATIC
  ${ROBIN_ROOT}/src/core.cpp
  ${ROBIN_ROOT}/src/graph_core.cpp
  ${ROBIN_ROOT}/src/graph.cpp
  ${ROBIN_ROOT}/src/graph_io.cpp
  ${ROBIN_ROOT}/src/graph_solvers.cpp
  ${ROBIN_ROOT}/src/pkc.cpp
  ${ROBIN_ROOT}/src/problems.cpp
  ${ROBIN_ROOT}/src/robin.cpp
  ${ROBIN_ROOT}/src/utils.cpp
)
set_target_properties(robin_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(robin_core
  PUBLIC
    ${ROBIN_ROOT}/include
    ${THIRD_PARTY_ROOT}/xenium
)
target_link_libraries(robin_core
  PUBLIC Eigen3::Eigen
  PRIVATE pmc_core OpenMP::OpenMP_CXX
)

# Build only the KISS-Matcher core required by this node. Upstream ROS, Python,
# examples and pose-graph/SAM targets are intentionally not vendored.
set(KISS_MATCHER_ROOT "${THIRD_PARTY_ROOT}/kiss_matcher")
add_library(kiss_matcher_core STATIC
  ${KISS_MATCHER_ROOT}/core/kiss_matcher/ROBINMatching.cpp
  ${KISS_MATCHER_ROOT}/core/kiss_matcher/FasterPFH.cpp
  ${KISS_MATCHER_ROOT}/core/kiss_matcher/KISSMatcher.cpp
  ${KISS_MATCHER_ROOT}/core/kiss_matcher/GncSolver.cpp
)
set_target_properties(kiss_matcher_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(kiss_matcher_core PUBLIC ${KISS_MATCHER_ROOT}/core)
target_link_libraries(kiss_matcher_core
  PUBLIC
    Eigen3::Eigen
    ${FLANN_CPP_LIBRARY}
    robin_core
    TBB::tbb
    OpenMP::OpenMP_CXX
)
target_include_directories(kiss_matcher_core PUBLIC ${FLANN_INCLUDE_DIR})

# Use the pinned local small_gicp source. This keeps builds offline and avoids
# repeated FetchContent downloads from every relocalization package.
set(BUILD_HELPER ON CACHE BOOL "Build small_gicp helper library" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable small_gicp tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable small_gicp examples" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable small_gicp benchmarks" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Disable small_gicp Python bindings" FORCE)
set(BUILD_WITH_TBB OFF CACHE BOOL "Use OpenMP implementation only" FORCE)
set(BUILD_WITH_PCL OFF CACHE BOOL "PCL adapters are header-only for this node" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/small_gicp
  ${CMAKE_CURRENT_BINARY_DIR}/small_gicp
  EXCLUDE_FROM_ALL
)

add_library(global_kiss_matcher_relocalization_node SHARED
  src/global_kiss_matcher_relocalization.cpp
)
target_include_directories(global_kiss_matcher_relocalization_node
  PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)
target_compile_options(global_kiss_matcher_relocalization_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(global_kiss_matcher_relocalization_node
  ${PCL_LIBRARIES}
  small_gicp
  kiss_matcher_core
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(global_kiss_matcher_relocalization_node
  geometry_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_eigen
  tf2_ros
)

rclcpp_components_register_node(global_kiss_matcher_relocalization_node
  PLUGIN "global_kiss_matcher_relocalization::GlobalKissMatcherRelocalizationNode"
  EXECUTABLE global_kiss_matcher_relocalization_exec
)

install(TARGETS global_kiss_matcher_relocalization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()