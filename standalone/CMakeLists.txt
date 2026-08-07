cmake_minimum_required(VERSION 3.10)
project(fastlio_standalone)

set(CMAKE_CXX_STANDARD 17) # mcap headers use std::optional/string_view/structured bindings
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(yaml-cpp CONFIG REQUIRED)

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../libs/ikdtree" "${CMAKE_CURRENT_BINARY_DIR}/ikdtree")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../libs/fastlio" "${CMAKE_CURRENT_BINARY_DIR}/fastlio")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/third_party/mcap" "${CMAKE_CURRENT_BINARY_DIR}/mcap")

add_executable(fastlio_standalone
  main.cpp
  config.cpp
  mcap_reader.cpp
  viz/pcl_visualizer.cpp
)
# fastlio's PUBLIC usage requirements (PCL include dirs/libs, Eigen, cxx_std_14)
# propagate transitively, so PCL (including visualization) doesn't need its
# own find_package() call here.
target_link_libraries(fastlio_standalone PRIVATE fastlio ikdtree mcap yaml-cpp::yaml-cpp)

# Dev/regression-test tool -- synthesizes an mcap file for exercising the
# decode path without a real recording (see tools/gen_test_mcap.cpp).
add_executable(fastlio_gen_test_mcap tools/gen_test_mcap.cpp)
target_link_libraries(fastlio_gen_test_mcap PRIVATE mcap)
