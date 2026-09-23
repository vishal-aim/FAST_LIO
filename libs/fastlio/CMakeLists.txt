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
# CMakeLists.txt set for laserMapping.cpp's h_share_model. MP_EN turns on a
# #pragma omp parallel for over LioCore::hShareModelImpl's per-point
# matching loop (nearest-neighbor search + plane fit against the ikd-Tree).
#
# The x86/amd64-only check below is intentionally preserved rather than also
# enabling MP_EN on arm64 (e.g. Apple Silicon), even though OpenMP works
# fine there and this repo already links libomp for that architecture too.
# Measured on a real 15GB rosbag2 recording: enabling it on an arm64 machine
# gave a real ~31% wall-clock speedup (89s -> 61s on a 60s slice) but also
# made runs non-deterministic by ~5cm of final-position drift over that same
# window -- the parallel matching loop races the ikd-Tree's own background
# rebuild thread (see ikd_Tree.cpp's start_thread()), which is designed to
# tolerate concurrent search-during-rebuild but not to give every query a
# consistent tree snapshot when queries themselves speed up. Repeatable runs
# currently matter more than the speedup, so this stays off until that's no
# longer true -- if you do want it, drop the architecture check and keep
# just the FASTLIO_NPROC tiers below.
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
