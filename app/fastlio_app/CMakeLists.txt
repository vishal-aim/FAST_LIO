# The standalone (ROS-free) FAST-LIO executable.
find_package(yaml-cpp CONFIG REQUIRED)

add_executable(fastlio_app
  main.cpp
  config.cpp
  bag_replay.cpp
)
# readers' own PUBLIC include dir is app/ itself, so "readers/..." keeps
# resolving from this subfolder exactly like it did when these files lived
# directly under app/ -- no extra target_include_directories() needed here.
#
# fastlio's PUBLIC usage requirements (PCL include dirs/libs, Eigen, cxx_std_14)
# propagate transitively, so PCL (including visualization) doesn't need its
# own find_package() call here.
target_link_libraries(fastlio_app PRIVATE readers fastlio ikdtree viz map
                                           yaml-cpp::yaml-cpp)
