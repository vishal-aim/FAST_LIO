# Standalone Patchwork++ ground-segmentation viewer: reads one lidar topic
# from any BagSource and streams ground/non-ground per scan to Rerun. No
# IMU, no motion compensation, no map accumulation -- ground segmentation is
# per-scan in the sensor's own frame, so this app is much smaller than
# fastlio_app and deliberately doesn't share its decode step.
add_executable(patchwork_app main.cpp)
# readers' own PUBLIC include dir is app/ itself, so "readers/..." resolves
# the same way it does from app/fastlio_app/. patchworkpp::ground_seg_cores
# brings <patchwork/patchworkpp.h> and Eigen3::Eigen transitively; rerun_sdk
# brings <rerun.hpp>.
target_link_libraries(patchwork_app PRIVATE readers patchworkpp::ground_seg_cores rerun_sdk)
