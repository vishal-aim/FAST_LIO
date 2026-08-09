# The standalone (ROS-free) FAST-LIO executable and its dev/test tools.
# libs/ikdtree, libs/fastlio, libs/viz, libs/map, and third_party/mcap are
# all added by the root CMakeLists.txt before this subdirectory, so this
# file just links against those already-defined targets.
find_package(yaml-cpp CONFIG REQUIRED)
find_package(SQLite3 REQUIRED)  # rosbag2 .db3 files are plain SQLite3 databases

add_executable(fastlio_app
  main.cpp
  config.cpp
  bag_replay.cpp
  readers/bag_source.cpp
  readers/mcap_bag_source.cpp
  readers/ros2_sqlite_bag_source.cpp
)
# fastlio's PUBLIC usage requirements (PCL include dirs/libs, Eigen, cxx_std_14)
# propagate transitively, so PCL (including visualization) doesn't need its
# own find_package() call here.
target_link_libraries(fastlio_app PRIVATE fastlio ikdtree viz map mcap
                                           yaml-cpp::yaml-cpp SQLite3::SQLite3)

# Dev/regression-test tools -- synthesize a bag file for exercising the
# decode path without a real recording (see tools/gen_test_*.cpp).
add_executable(fastlio_gen_test_mcap tools/gen_test_mcap.cpp)
target_link_libraries(fastlio_gen_test_mcap PRIVATE mcap)

add_executable(fastlio_gen_test_ros2_bag tools/gen_test_ros2_bag.cpp)
target_link_libraries(fastlio_gen_test_ros2_bag PRIVATE SQLite3::SQLite3)
