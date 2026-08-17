# Thin shared layer on top of aimcap::reader -- just the generic
# PointField byte-reading helper (point_field_reader.h) both app/fastlio_app
# and app/patchwork_app need; everything sensor/app-specific stays in each
# app rather than growing this into another bag-format abstraction.
add_library(readers INTERFACE)
# Parent dir as the include root (matches libs/ikdtree's own pattern) so
# callers write #include "readers/point_field_reader.h" regardless of which
# app/*_app subfolder they live in.
target_include_directories(readers INTERFACE "${CMAKE_CURRENT_LIST_DIR}/..")
target_link_libraries(readers INTERFACE aimcap::reader)
