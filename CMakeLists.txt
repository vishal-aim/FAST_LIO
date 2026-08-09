# Plain (non-catkin) CMake project. The ROS1 catkin build (fastlio_mapping,
# src/laserMapping.cpp) has been dropped from this file -- it isn't
# build-tested against a real ROS install, and the actively-developed,
# actively-run part of this repo is the standalone (ROS-free) app/. Nothing
# here references src/laserMapping.cpp, package.xml, config/*.yaml, or
# launch/*.launch; they stay in the repo for reference only.
cmake_minimum_required(VERSION 3.10)
project(fast_lio)

set(CMAKE_CXX_STANDARD 17)  # app/ and third_party/mcap need 17; libs/{fastlio,ikdtree,viz}
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # pin themselves to 14 via target_compile_features regardless
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# On macOS, CMake's FindOpenMP module can't locate Homebrew's libomp on its
# own (no default search path covers it), and PCL's own internal OpenMP probe
# (triggered by find_package(PCL) below) caches that failure for the rest of
# the build -- silently breaking fastlio's unconditional #include <omp.h>
# even when MP_EN itself is off. Hinting CMAKE_PREFIX_PATH here, before any
# find_package(PCL)/find_package(OpenMP) call runs, is a no-op everywhere
# else (empty/unset if brew or libomp isn't present).
if(APPLE)
  execute_process(COMMAND brew --prefix libomp
                   OUTPUT_VARIABLE HOMEBREW_LIBOMP_PREFIX
                   OUTPUT_STRIP_TRAILING_WHITESPACE
                   ERROR_QUIET)
  if(HOMEBREW_LIBOMP_PREFIX)
    list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_LIBOMP_PREFIX}")
  endif()
endif()

add_subdirectory(libs/ikdtree)
add_subdirectory(libs/fastlio)
add_subdirectory(libs/viz)
add_subdirectory(libs/map)
add_subdirectory(third_party/mcap)
add_subdirectory(app)
