# Top-level router for app/: each algorithm gets its own executable in its
# own subfolder, built on top of the shared readers/ library. libs/{ikdtree,
# fastlio,viz,map}, third_party/aimcap, and third_party/rerun_cpp are all
# added by the root CMakeLists.txt before this subdirectory, so the
# subfolders below just link against those already-defined targets.
add_subdirectory(readers)
add_subdirectory(fastlio_app)
add_subdirectory(patchwork_app)
