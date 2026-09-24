#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <aimcap/reader.hpp>
#include <aimcap/types.hpp>

#include <fastlio/common_lib.h>
#include <fastlio/lio_core.h>
#include <fastlio/packet_sync.h>
#include <fastlio/types.h>
#include <map/incremental_voxel_map.h>
#include <pcl/io/pcd_io.h>

#include "config.h"
#include "point_field_reader.h"

namespace
{

std::atomic<bool> g_shutdown{false};
aimcap::Reader *g_active_reader = nullptr;

void sigHandler(int)
{
  g_shutdown.store(true);
  if (g_active_reader)
  {
    g_active_reader->Stop();
  }
}

struct ProgramArgs
{
  std::string input_mcap;
  std::string output_dir = "./results";
  std::string config_path = "config/ouster_os1_aim.yaml";
  std::string lidar_topic_override;
  std::string imu_topic_override;
  double max_duration = 0.0;
  double start_offset = 0.0;
  double playback_rate = 0.0;
  double map_voxel_size = 0.5;
  bool save_map = true;
};

void printUsage(const char *prog)
{
  std::cout << "Usage: " << prog << " [options]\n\n"
            << "Options:\n"
            << "  -i, --input <file>       Input MCAP recording file (required)\n"
            << "  -o, --output <dir>       Output directory for trajectory (default: ./results)\n"
            << "  -c, --config <file>      Configuration YAML file (default: config/ouster_os1_aim.yaml)\n"
            << "  --lidar <topic>          Override LiDAR topic name\n"
            << "  --imu <topic>            Override IMU topic name\n"
            << "  -d, --duration <sec>     Maximum duration in seconds to process (default: full file)\n"
            << "  -s, --start <sec>        Offset in seconds from start of bag to begin processing\n"
            << "  -r, --rate <float>       Playback speed multiplier (0 = unlimited / max speed, 1.0 = real-time)\n"
            << "  --map-voxel-size <m>     Voxel size in meters for map accumulation (default: 0.5)\n"
            << "  --no-save-map            Disable saving accumulated map scans.pcd\n"
            << "  --headless               Run headless (always enabled, provided for compatibility)\n"
            << "  -h, --help               Display this help\n";
}

bool parseArgs(int argc, char **argv, ProgramArgs &args)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if ((arg == "-i" || arg == "--input") && i + 1 < argc)
    {
      args.input_mcap = argv[++i];
    }
    else if ((arg == "-o" || arg == "--output") && i + 1 < argc)
    {
      args.output_dir = argv[++i];
    }
    else if ((arg == "-c" || arg == "--config") && i + 1 < argc)
    {
      args.config_path = argv[++i];
    }
    else if (arg == "--lidar" && i + 1 < argc)
    {
      args.lidar_topic_override = argv[++i];
    }
    else if (arg == "--imu" && i + 1 < argc)
    {
      args.imu_topic_override = argv[++i];
    }
    else if ((arg == "-d" || arg == "--duration") && i + 1 < argc)
    {
      args.max_duration = std::stod(argv[++i]);
    }
    else if ((arg == "-s" || arg == "--start") && i + 1 < argc)
    {
      args.start_offset = std::stod(argv[++i]);
    }
    else if ((arg == "-r" || arg == "--rate") && i + 1 < argc)
    {
      args.playback_rate = std::stod(argv[++i]);
    }
    else if (arg == "--map-voxel-size" && i + 1 < argc)
    {
      args.map_voxel_size = std::stod(argv[++i]);
    }
    else if (arg == "--no-save-map")
    {
      args.save_map = false;
    }
    else if (arg == "--headless")
    {
      // Already headless
    }
    else if (arg == "-h" || arg == "--help")
    {
      printUsage(argv[0]);
      std::exit(0);
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << "\n";
      return false;
    }
  }
  return !args.input_mcap.empty();
}

PointCloudXYZI::Ptr toWorldFrame(const state_ikfom &s, const PointCloudXYZI::Ptr &bodyCloud)
{
  PointCloudXYZI::Ptr world(new PointCloudXYZI(bodyCloud->size(), 1));
  for (size_t i = 0; i < bodyCloud->size(); ++i)
  {
    const PointType &pi = bodyCloud->points[i];
    V3D p_body(pi.x, pi.y, pi.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    PointType &po = world->points[i];
    po.x = p_global(0);
    po.y = p_global(1);
    po.z = p_global(2);
    po.intensity = pi.intensity;
  }
  return world;
}

void decodePointCloud(const aimcap::DecodedPointCloud &cloud,
                      const fastlio::StandaloneConfig &cfg,
                      PointCloudXYZI::Ptr &out)
{
  const auto *fx = fastlio::findField(cloud.fields, "x");
  const auto *fy = fastlio::findField(cloud.fields, "y");
  const auto *fz = fastlio::findField(cloud.fields, "z");
  const auto *fi = fastlio::findField(cloud.fields, "intensity");
  if (!fi) fi = fastlio::findField(cloud.fields, "reflectivity");
  const auto *ft = fastlio::findField(cloud.fields, "t");
  if (!ft) ft = fastlio::findField(cloud.fields, "time");

  if (!fx || !fy || !fz)
  {
    std::cerr << "[warn] Point cloud missing x, y, or z field!\n";
    return;
  }

  const size_t total_pts = cloud.width * cloud.height;
  out->points.reserve(total_pts / std::max(1, cfg.point_filter_num));

  const double blind_sq = cfg.blind * cfg.blind;
  const double det_sq = static_cast<double>(cfg.lio.det_range) * cfg.lio.det_range;

  // Determine time scale to convert to milliseconds
  // timestamp_unit: 0=SEC, 1=MS, 2=US, 3=NS
  double time_to_ms = 1e-6; // default nanoseconds to milliseconds
  if (cfg.timestamp_unit == 0) time_to_ms = 1e3;
  else if (cfg.timestamp_unit == 1) time_to_ms = 1.0;
  else if (cfg.timestamp_unit == 2) time_to_ms = 1e-3;
  else if (cfg.timestamp_unit == 3) time_to_ms = 1e-6;

  size_t valid_pt_idx = 0;
  for (size_t i = 0; i < total_pts; ++i)
  {
    const uint8_t *base = cloud.data.data() + i * cloud.point_step;
    float x = static_cast<float>(fastlio::readFieldAsDouble(base, *fx));
    float y = static_cast<float>(fastlio::readFieldAsDouble(base, *fy));
    float z = static_cast<float>(fastlio::readFieldAsDouble(base, *fz));

    if (x == 0.0f && y == 0.0f && z == 0.0f) continue;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    double d2 = x * x + y * y + z * z;
    if (d2 < blind_sq || d2 > det_sq) continue;

    if (cfg.point_filter_num > 1 && (valid_pt_idx++ % cfg.point_filter_num != 0)) continue;

    float intensity = fi ? static_cast<float>(fastlio::readFieldAsDouble(base, *fi)) : 0.0f;
    double t_raw = ft ? fastlio::readFieldAsDouble(base, *ft) : 0.0;
    float t_ms = static_cast<float>(t_raw * time_to_ms);

    PointType pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = intensity;
    pt.curvature = t_ms; // FAST-LIO stores per-point relative timestamp in curvature (ms)
    pt.normal_x = 0;
    pt.normal_y = 0;
    pt.normal_z = 0;
    out->points.push_back(pt);
  }
}

} // namespace

int main(int argc, char **argv)
{
  std::signal(SIGINT, sigHandler);
  std::signal(SIGTERM, sigHandler);

  ProgramArgs args;
  if (!parseArgs(argc, argv, args))
  {
    printUsage(argv[0]);
    return 1;
  }

  if (!std::filesystem::exists(args.config_path))
  {
    if (std::filesystem::exists("/opt/fastlio/" + args.config_path))
    {
      args.config_path = "/opt/fastlio/" + args.config_path;
    }
  }

  std::cout << "========================================================\n"
            << " FAST-LIO Standalone MCAP Runner\n"
            << "========================================================\n"
            << " Input MCAP  : " << args.input_mcap << "\n"
            << " Output Dir  : " << args.output_dir << "\n"
            << " Config File : " << args.config_path << "\n";

  fastlio::StandaloneConfig cfg;
  try
  {
    cfg = fastlio::loadYamlConfig(args.config_path);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to load config '" << args.config_path << "': " << e.what() << "\n";
    return 1;
  }

  if (!args.lidar_topic_override.empty()) cfg.lid_topic = args.lidar_topic_override;
  if (!args.imu_topic_override.empty()) cfg.imu_topic = args.imu_topic_override;

  std::cout << " LiDAR Topic : " << cfg.lid_topic << "\n"
            << " IMU Topic   : " << cfg.imu_topic << "\n"
            << " PointFilter : " << cfg.point_filter_num << " (blind=" << cfg.blind << ", det_range=" << cfg.lio.det_range << ")\n"
            << " Map Config  : cube_side_length=" << cfg.lio.cube_side_length << ", map_res=" << cfg.lio.filter_size_map_min << ", max_iter=" << cfg.lio.max_iterations << "\n"
            << " Extrinsics  : T=[" << cfg.lio.extrinsic_T.transpose() << "]\n"
            << "               R=\n" << cfg.lio.extrinsic_R << "\n"
            << "========================================================\n";

  std::filesystem::create_directories(args.output_dir);
  std::string tum_path = args.output_dir + "/trajectory_tum.txt";
  std::ofstream tum_file(tum_path);
  if (!tum_file.is_open())
  {
    std::cerr << "Failed to open output file: " << tum_path << "\n";
    return 1;
  }
  tum_file << std::fixed << std::setprecision(6);

  std::string full_path = args.output_dir + "/trajectory_full.txt";
  std::ofstream full_file(full_path);
  if (full_file.is_open())
  {
    full_file << std::fixed << std::setprecision(6);
  }

  std::unique_ptr<aimcap::Reader> reader;
  std::optional<uint64_t> total_lidar_scans;
  try
  {
    reader = std::make_unique<aimcap::Reader>(args.input_mcap, /*add_column_timestamps=*/true, "t");
    total_lidar_scans = reader->MessageCount(cfg.lid_topic);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to open MCAP file '" << args.input_mcap << "': " << e.what() << "\n";
    return 1;
  }
  g_active_reader = reader.get();

  if (total_lidar_scans)
  {
    std::cout << "Found " << *total_lidar_scans << " LiDAR scans on " << cfg.lid_topic << "\n";
  }

  // Heap-allocate LioCore (embeds ikd-tree buffers)
  auto lio = std::make_unique<fastlio::LioCore>(cfg.lio);

  fastlio::PacketSync sync;
  sync.setLidarType(cfg.lio.lidar_type);

  double voxel_size = args.map_voxel_size > 0 ? args.map_voxel_size : cfg.lio.filter_size_map_min;
  fastlio::IncrementalVoxelMap voxel_map(voxel_size);

  size_t scans_seen = 0;
  size_t imu_seen = 0;
  size_t frame_count = 0;
  double bag_start_time = -1.0;
  auto wall_start = std::chrono::steady_clock::now();

  auto reportProgress = [&]() {
    if (total_lidar_scans && *total_lidar_scans > 0)
    {
      double pct = 100.0 * static_cast<double>(scans_seen) / static_cast<double>(*total_lidar_scans);
      std::cout << "[progress] scan " << scans_seen << "/" << *total_lidar_scans << " ("
                << std::fixed << std::setprecision(1) << pct << "%), valid frames=" << frame_count
                << ", map voxels=" << voxel_map.voxelCount() << std::endl;
    }
    else
    {
      std::cout << "[progress] scan " << scans_seen << ", valid frames=" << frame_count
                << ", map voxels=" << voxel_map.voxelCount() << std::endl;
    }
  };

  auto onMeasurement = [&](const MeasureGroup &meas) {
    if (g_shutdown.load()) return;

    fastlio::LioCore::FrameResult res = lio->processFrame(meas);
    if (!res.ok)
    {
      if (scans_seen % 50 == 0) reportProgress();
      return;
    }

    Eigen::Quaterniond q(res.state.rot);

    // Standard 8-column TUM format: timestamp tx ty tz qx qy qz qw
    tum_file << res.time << " "
             << res.state.pos.x() << " " << res.state.pos.y() << " " << res.state.pos.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    tum_file.flush();

    if (full_file.is_open())
    {
      full_file << res.time << " "
                << res.state.pos.x() << " " << res.state.pos.y() << " " << res.state.pos.z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
                << res.state.vel.x() << " " << res.state.vel.y() << " " << res.state.vel.z() << " "
                << res.angvel.x() << " " << res.angvel.y() << " " << res.angvel.z() << "\n";
      full_file.flush();
    }

    if (args.save_map)
    {
      PointCloudXYZI::Ptr worldCloud = toWorldFrame(res.state, res.down_body);
      voxel_map.addCloud(*worldCloud);
    }

    frame_count++;
    if (scans_seen % 50 == 0 || frame_count % 50 == 0)
    {
      reportProgress();
    }

    if (cfg.runtime_pos_log_enable)
    {
      std::cout << "[frame " << frame_count << "] t=" << res.time
                << " pos=(" << res.state.pos.x() << ", " << res.state.pos.y() << ", " << res.state.pos.z() << ")"
                << " match=" << res.match_time << " solve=" << res.solve_time
                << " incr=" << res.kdtree_incremental_time << " eff_pts=" << res.effect_feat_num << "\n";
    }
  };

  MeasureGroup meas;
  auto drainReady = [&]() {
    while (sync.nextMeasurement(meas) && !g_shutdown.load())
    {
      onMeasurement(meas);
    }
  };

  // Register IMU callback
  reader->OnImu(cfg.imu_topic, [&](const std::string &, const aimcap::DecodedImu &imu) {
    if (g_shutdown.load()) { reader->Stop(); return; }

    const double stamp = imu.time_ns * 1e-9;
    if (bag_start_time < 0.0) bag_start_time = stamp;

    const double rel_time = stamp - bag_start_time;
    if (args.start_offset > 0.0 && rel_time < args.start_offset) return;
    if (args.max_duration > 0.0 && (rel_time - args.start_offset) > args.max_duration)
    {
      g_shutdown.store(true);
      reader->Stop();
      return;
    }

    fastlio::ImuSample sample;
    sample.timestamp = stamp - cfg.time_offset_lidar_to_imu;
    sample.acc = Eigen::Vector3d(imu.linear_acceleration_mps2[0],
                                 imu.linear_acceleration_mps2[1],
                                 imu.linear_acceleration_mps2[2]);
    sample.gyro = Eigen::Vector3d(imu.angular_velocity_radps[0],
                                  imu.angular_velocity_radps[1],
                                  imu.angular_velocity_radps[2]);
    sync.pushImu(sample);
    imu_seen++;
    drainReady();
  });

  // Register PointCloud callback
  reader->OnPointCloud(cfg.lid_topic, [&](const std::string &, const aimcap::DecodedPointCloud &cloud) {
    if (g_shutdown.load()) { reader->Stop(); return; }

    const double stamp = cloud.stamp_sec + cloud.stamp_nsec * 1e-9;
    if (bag_start_time < 0.0) bag_start_time = stamp;

    const double rel_time = stamp - bag_start_time;
    if (args.start_offset > 0.0 && rel_time < args.start_offset) return;
    if (args.max_duration > 0.0 && (rel_time - args.start_offset) > args.max_duration)
    {
      g_shutdown.store(true);
      reader->Stop();
      return;
    }

    // Playback rate throttling if requested
    if (args.playback_rate > 0.0)
    {
      auto wall_now = std::chrono::steady_clock::now();
      double wall_elapsed = std::chrono::duration<double>(wall_now - wall_start).count();
      double target_wall = (rel_time - args.start_offset) / args.playback_rate;
      if (target_wall > wall_elapsed)
      {
        std::this_thread::sleep_for(std::chrono::duration<double>(target_wall - wall_elapsed));
      }
    }

    PointCloudXYZI::Ptr pcl_cloud(new PointCloudXYZI());
    decodePointCloud(cloud, cfg, pcl_cloud);

    sync.pushLidar(pcl_cloud, stamp);
    scans_seen++;
    drainReady();
  });

  std::cout << "Starting replay...\n";
  try
  {
    reader->Run();
  }
  catch (const std::exception &e)
  {
    std::cerr << "MCAP replay error: " << e.what() << "\n";
  }

  tum_file.close();
  if (full_file.is_open()) full_file.close();
  g_active_reader = nullptr;

  // Also create traj.tum symlink / copy for convenience
  std::string alt_tum_path = args.output_dir + "/traj.tum";
  std::error_code ec;
  std::filesystem::remove(alt_tum_path, ec);
  std::filesystem::copy_file(tum_path, alt_tum_path, std::filesystem::copy_options::overwrite_existing, ec);

  std::cout << "\n========================================================\n"
            << " Processing Complete\n"
            << " Total Scans Seen : " << scans_seen << "\n"
            << " Total IMU Samples: " << imu_seen << "\n"
            << " Poses Estimated  : " << frame_count << "\n"
            << " Trajectory File  : " << tum_path << "\n";

  if (args.save_map && voxel_map.voxelCount() > 0)
  {
    std::string pcd_path = args.output_dir + "/scans.pcd";
    auto final_map = voxel_map.toCloud();
    pcl::io::savePCDFileBinary(pcd_path, *final_map);
    std::cout << " Accumulated Map  : " << pcd_path << " (" << final_map->size() << " points)\n";
  }
  std::cout << "========================================================\n";

  return 0;
}
