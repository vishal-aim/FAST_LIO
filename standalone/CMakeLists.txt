cmake_minimum_required(VERSION 3.10)
project(fastlio_standalone)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../libs/ikdtree" "${CMAKE_CURRENT_BINARY_DIR}/ikdtree")
add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../libs/fastlio" "${CMAKE_CURRENT_BINARY_DIR}/fastlio")

add_executable(fastlio_standalone main.cpp)
target_link_libraries(fastlio_standalone PRIVATE fastlio ikdtree)
