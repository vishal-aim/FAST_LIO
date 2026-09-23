cmake_minimum_required(VERSION 3.16)
project(fast_lio LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenMP QUIET)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# Subdirectories
add_subdirectory(libs/ikdtree)
add_subdirectory(libs/fastlio)
add_subdirectory(libs/map)

# aimcap MCAP reader library
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/aimcap/CMakeLists.txt")
  set(AIMCAP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  add_subdirectory(thirdparty/aimcap EXCLUDE_FROM_ALL)
endif()

# Standalone MCAP executable
add_executable(fastlio_mcap
  src/fastlio_mcap.cpp
  src/config.cpp
)

target_include_directories(fastlio_mcap PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(fastlio_mcap PRIVATE
  fastlio
  ikdtree
  map
  aimcap::reader
  ${YAML_CPP_TARGET}
)

set_target_properties(fastlio_mcap PROPERTIES
  INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;/usr/local/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)