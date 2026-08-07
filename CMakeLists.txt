cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_subdirectory(libs/ikdtree)
add_subdirectory(libs/fastlio)

add_executable(fastlio_mapping src/laserMapping.cpp)
target_include_directories(fastlio_mapping PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(fastlio_mapping fastlio ikdtree ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# Only laserMapping.cpp uses ROOT_DIR/DEBUG_FILE_DIR (runtime_pos_log_enable's
# debug dump); scoped here, not via a project-wide add_definitions(), so it
# doesn't leak into the fastlio/ikdtree library targets added above, which
# must not be tied to one repo's absolute checkout path.
target_compile_definitions(fastlio_mapping PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
