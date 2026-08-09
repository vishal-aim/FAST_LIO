#ifndef FASTLIO_APP_CONFIG_H
#define FASTLIO_APP_CONFIG_H

#include <string>
#include <fastlio/lio_core.h>

namespace fastlio_app
{

// Everything the ROS1 node loaded via nh.param<...>(...), read from the same
// config/*.yaml files instead (they're already plain nested YAML -- ROS's
// `rosparam load` just dumps YAML onto the param server, it doesn't reshape
// it). A few tuning knobs (max_iteration, filter_size_surf/map,
// cube_side_length, point_filter_num, feature_extract_enable,
// runtime_pos_log_enable) live in each launch file's <param> tags rather
// than the yaml upstream; here they're optional top-level yaml keys that
// fall back to the same hardcoded defaults laserMapping.cpp used.
struct StandaloneConfig
{
  fastlio::LioConfig lio;

  // Preprocess (fastlio::Preprocess) settings.
  int scan_line = 16;
  int scan_rate = 10;
  int timestamp_unit = 2;  // US, see fastlio/preprocess.h::TIME_UNIT
  double blind = 0.01;
  int point_filter_num = 2;
  bool feature_extract_enable = false;

  // mcap topic selection.
  std::string lid_topic = "/velodyne_points";
  std::string imu_topic = "/imu/data";
  double time_offset_lidar_to_imu = 0.0;

  bool runtime_pos_log_enable = false;
};

// Throws std::runtime_error on a missing file or a YAML parse error.
StandaloneConfig loadYamlConfig(const std::string &path);

}  // namespace fastlio_app

#endif
