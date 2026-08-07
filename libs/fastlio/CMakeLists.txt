# Core FAST-LIO algorithm library: ESIKF state estimation, IMU processing,
# and lidar preprocessing. Contains no ROS or vendor-driver (livox_ros_driver)
# includes anywhere -- boundary types are plain structs (see include/fastlio/types.h)
# or PCL point types. Linked by both the ROS1 node (src/laserMapping.cpp) and
# the standalone MCAP executable (standalone/).
set(FASTLIO_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenMP QUIET)

if(NOT TARGET ikdtree)
  add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/../ikdtree" "${CMAKE_CURRENT_BINARY_DIR}/ikdtree")
endif()

add_library(fastlio STATIC
  src/imu_process.cpp
  src/preprocess.cpp
  src/packet_sync.cpp
  src/lio_core.cpp
)

# Matches the CPU-count-based multi-threading knobs the original top-level
# CMakeLists.txt set for laserMapping.cpp's h_share_model.
include(ProcessorCount)
ProcessorCount(FASTLIO_NPROC)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" AND FASTLIO_NPROC GREATER 3)
  if(FASTLIO_NPROC GREATER 4)
    target_compile_definitions(fastlio PUBLIC MP_EN MP_PROC_NUM=3)
  else()
    target_compile_definitions(fastlio PUBLIC MP_EN MP_PROC_NUM=2)
  endif()
else()
  target_compile_definitions(fastlio PUBLIC MP_PROC_NUM=1)
endif()

target_include_directories(fastlio PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/include"
)
# use-ikfom.hpp / lio_core.h pull in the vendored, header-only IKFoM_toolkit
# and ikd-Tree, which still live at the repo root under include/.
target_include_directories(fastlio SYSTEM PUBLIC
  "${FASTLIO_REPO_ROOT}/include"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(fastlio PUBLIC ikdtree ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio PUBLIC OpenMP::OpenMP_CXX)
endif()
target_compile_features(fastlio PUBLIC cxx_std_14)
