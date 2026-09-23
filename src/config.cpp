#include "config.h"

#include <iostream>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace fastlio
{

namespace
{

template <typename T>
T getOr(const YAML::Node &node, const char *key, T fallback)
{
  if (node && node[key])
  {
    try
    {
      return node[key].as<T>();
    }
    catch (const std::exception &e)
    {
      std::cerr << "[warn] Failed to parse key '" << key << "': " << e.what() << ", using fallback\n";
    }
  }
  return fallback;
}

} // namespace

StandaloneConfig loadYamlConfig(const std::string &path)
{
  YAML::Node root = YAML::LoadFile(path);

  StandaloneConfig cfg;

  YAML::Node common = root["common"];
  cfg.lid_topic = getOr<std::string>(common, "lid_topic", cfg.lid_topic);
  cfg.imu_topic = getOr<std::string>(common, "imu_topic", cfg.imu_topic);
  cfg.time_offset_lidar_to_imu = getOr<double>(common, "time_offset_lidar_to_imu", cfg.time_offset_lidar_to_imu);

  YAML::Node preprocess = root["preprocess"];
  cfg.lio.lidar_type = getOr<int>(preprocess, "lidar_type", cfg.lio.lidar_type);
  cfg.scan_line = getOr<int>(preprocess, "scan_line", cfg.scan_line);
  cfg.scan_rate = getOr<int>(preprocess, "scan_rate", cfg.scan_rate);
  cfg.timestamp_unit = getOr<int>(preprocess, "timestamp_unit", cfg.timestamp_unit);
  cfg.blind = getOr<double>(preprocess, "blind", cfg.blind);

  YAML::Node mapping = root["mapping"];
  cfg.lio.gyr_cov = getOr<double>(mapping, "gyr_cov", cfg.lio.gyr_cov);
  cfg.lio.acc_cov = getOr<double>(mapping, "acc_cov", cfg.lio.acc_cov);
  cfg.lio.b_gyr_cov = getOr<double>(mapping, "b_gyr_cov", cfg.lio.b_gyr_cov);
  cfg.lio.b_acc_cov = getOr<double>(mapping, "b_acc_cov", cfg.lio.b_acc_cov);
  cfg.lio.det_range = getOr<float>(mapping, "det_range", cfg.lio.det_range);
  cfg.lio.extrinsic_est_en = getOr<bool>(mapping, "extrinsic_est_en", cfg.lio.extrinsic_est_en);

  if (mapping && mapping["extrinsic_T"])
  {
    auto t = mapping["extrinsic_T"].as<std::vector<double>>();
    if (t.size() != 3) throw std::runtime_error("mapping/extrinsic_T must have 3 elements");
    cfg.lio.extrinsic_T = V3D(t[0], t[1], t[2]);
  }
  if (mapping && mapping["extrinsic_R"])
  {
    auto r = mapping["extrinsic_R"].as<std::vector<double>>();
    if (r.size() != 9) throw std::runtime_error("mapping/extrinsic_R must have 9 elements");
    cfg.lio.extrinsic_R << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
  }

  cfg.lio.max_iterations = getOr<int>(root, "max_iteration", cfg.lio.max_iterations);
  cfg.lio.filter_size_surf_min = getOr<double>(root, "filter_size_surf", cfg.lio.filter_size_surf_min);
  cfg.lio.filter_size_map_min = getOr<double>(root, "filter_size_map", cfg.lio.filter_size_map_min);
  cfg.lio.cube_side_length = getOr<double>(root, "cube_side_length", cfg.lio.cube_side_length);
  cfg.point_filter_num = getOr<int>(root, "point_filter_num", cfg.point_filter_num);
  cfg.feature_extract_enable = getOr<bool>(root, "feature_extract_enable", cfg.feature_extract_enable);
  cfg.runtime_pos_log_enable = getOr<bool>(root, "runtime_pos_log_enable", cfg.runtime_pos_log_enable);

  return cfg;
}

} // namespace fastlio
