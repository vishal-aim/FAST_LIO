cmake_minimum_required(VERSION 3.10)
project(fastlio_standalone)

set(CMAKE_CXX_STANDARD 17) # mcap headers use std::optional/string_view/structured bindings
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(yaml-cpp CONFIG REQUIRED)
find_package(SQLite3 REQUIRED) # rosbag2 .db3 files are plain SQLite3 databases

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../libs/ikdtree" "${CMAKE_CURRENT_BINARY_DIR}/ikdtree")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../libs/fastlio" "${CMAKE_CURRENT_BINARY_DIR}/fastlio")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/third_party/mcap" "${CMAKE_CURRENT_BINARY_DIR}/mcap")

add_executable(fastlio_standalone
  main.cpp
  config.cpp
  bag_source.cpp
  mcap_bag_source.cpp
  ros2_sqlite_bag_source.cpp
  bag_replay.cpp
  viz/pcl_visualizer.cpp
)
# fastlio's PUBLIC usage requirements (PCL include dirs/libs, Eigen, cxx_std_14)
# propagate transitively, so PCL (including visualization) doesn't need its
# own find_package() call here.
target_link_libraries(fastlio_standalone PRIVATE fastlio ikdtree mcap yaml-cpp::yaml-cpp SQLite3::SQLite3)

# Dev/regression-test tools -- synthesize a bag file for exercising the
# decode path without a real recording (see tools/gen_test_*.cpp).
add_executable(fastlio_gen_test_mcap tools/gen_test_mcap.cpp)
target_link_libraries(fastlio_gen_test_mcap PRIVATE mcap)

add_executable(fastlio_gen_test_ros2_bag tools/gen_test_ros2_bag.cpp)
target_link_libraries(fastlio_gen_test_ros2_bag PRIVATE SQLite3::SQLite3)
