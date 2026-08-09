# Visualization backends for the standalone app, kept as its own library
# (mirroring fastlio/ikdtree) so app code only depends on the abstract
# Visualizer interface, not a specific backend.
add_library(viz STATIC pcl_visualizer.cpp)
# Parent dir as the include root (matches libs/ikdtree's own pattern) so
# callers write #include <viz/visualizer.h>.
target_include_directories(viz PUBLIC "${CMAKE_CURRENT_LIST_DIR}/..")
# pcl::visualization::PCLVisualizer and Visualizer's interface types (V3D,
# PointCloudXYZI) both come from fastlio's own PUBLIC PCL/Eigen deps.
target_link_libraries(viz PUBLIC fastlio)
target_compile_features(viz PUBLIC cxx_std_14)
