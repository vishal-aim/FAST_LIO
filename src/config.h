#ifndef FASTLIO_CONFIG_H
#define FASTLIO_CONFIG_H

#include <string>
#include <fastlio/lio_core.h>

namespace fastlio
{

struct StandaloneConfig
{
  fastlio::LioConfig lio;

  // Preprocess settings
  int scan_line = 64;
  int scan_rate = 10;
  int timestamp_unit = 3;  // NS (see fastlio/preprocess.h::TIME_UNIT: 0=SEC, 1=MS, 2=US, 3=NS)
  double blind = 1.0;
  int point_filter_num = 2;
  bool feature_extract_enable = false;

  // Topics
  std::string lid_topic = "/ouster/points";
  std::string imu_topic = "/ouster/imu";
  double time_offset_lidar_to_imu = 0.0;

  bool runtime_pos_log_enable = false;
};

StandaloneConfig loadYamlConfig(const std::string &path);

} // namespace fastlio

#endif
