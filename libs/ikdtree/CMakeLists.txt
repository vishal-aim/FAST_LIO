# Wraps the vendored ikd-Tree git submodule (../../include/ikd-Tree) as a
# standalone CMake library target so it can be linked by both the ROS1
# catkin build and the standalone (non-ROS) build without either of them
# compiling include/ikd-Tree/ikd_Tree.cpp directly.
set(IKDTREE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../include/ikd-Tree")

find_package(PCL 1.8 REQUIRED)

add_library(ikdtree STATIC "${IKDTREE_ROOT}/ikd_Tree.cpp")
target_include_directories(ikdtree PUBLIC "${IKDTREE_ROOT}/..")
target_include_directories(ikdtree SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(ikdtree PUBLIC ${PCL_LIBRARIES})
target_compile_features(ikdtree PUBLIC cxx_std_14)
