# Header-only incremental voxel-grid map (see incremental_voxel_map.h for
# the centroid-drift rationale vs. re-running pcl::VoxelGrid).
add_library(map INTERFACE)
# Parent dir as the include root (matches libs/ikdtree's own pattern) so
# callers write #include <map/incremental_voxel_map.h>.
target_include_directories(map INTERFACE "${CMAKE_CURRENT_LIST_DIR}/..")
target_link_libraries(map INTERFACE fastlio)  # PointCloudXYZI/PointType
