#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <pcl/io/pcd_io.h>

#include <aimcap/reader.hpp>
#include <fastlio/lio_core.h>
#include <fastlio/packet_sync.h>
#include <fastlio/preprocess.h>

#include "bag_replay.h"
#include "config.h"
#include <map/incremental_voxel_map.h>
#include <viz/null_visualizer.h>
#include <viz/pcl_visualizer.h>
#include <viz/rerun_visualizer.h>
#include <viz/visualizer.h>

namespace
{

// How often (in synced measurements) to print a progress line.
constexpr size_t kProgressInterval = 50;

// Fallback if neither --map-voxel-size nor the config's filter_size_map_min
// resolve to something positive -- IncrementalVoxelMap needs a real leaf
// size, unlike the old "0 disables filtering" raw-accumulation mode.
constexpr double kDefaultMapVoxelSize = 0.1;

struct Args
{
  std::string bagPath;  // a single .mcap file, or a directory of numbered chunks
  std::string configPath;
  std::string outDir = ".";
  double mapVoxelSize = -1.0;  // <=0: use config's filter_size_map_min (or the hardcoded fallback)
  double startTimeOffset = 0.0;    // seconds from the bag's first message on lidTopic/imuTopic
  std::optional<double> duration;  // seconds to process, from startTimeOffset
  std::string viz = "pcl";  // "pcl", "rerun", or "none"
};

struct TrajPoint
{
  double time;
  V3D pos;
  Eigen::Quaterniond q;
  V3D vel;
  V3D angvel;
};

bool parseArgs(int argc, char **argv, Args &args)
{
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    if (arg == "--bag" && i + 1 < argc) args.bagPath = argv[++i];
    else if (arg == "--config" && i + 1 < argc) args.configPath = argv[++i];
    else if (arg == "--out" && i + 1 < argc) args.outDir = argv[++i];
    else if (arg == "--map-voxel-size" && i + 1 < argc) args.mapVoxelSize = std::stod(argv[++i]);
    else if (arg == "--start-time" && i + 1 < argc) args.startTimeOffset = std::stod(argv[++i]);
    else if (arg == "--duration" && i + 1 < argc) args.duration = std::stod(argv[++i]);
    else if (arg == "--viz" && i + 1 < argc) args.viz = argv[++i];
    else
    {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return false;
    }
  }
  return !args.bagPath.empty() && !args.configPath.empty();
}

PointCloudXYZI::Ptr toWorldFrame(const state_ikfom &s, const PointCloudXYZI::Ptr &bodyCloud)
{
  PointCloudXYZI::Ptr world(new PointCloudXYZI(bodyCloud->size(), 1));
  for (size_t i = 0; i < bodyCloud->size(); i++)
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

}  // namespace

int main(int argc, char **argv)
{
  Args args;
  if (!parseArgs(argc, argv, args))
  {
    std::cerr << "Usage: " << argv[0]
              << " --bag <file.mcap|chunk_dir> --config <config.yaml>"
                 " [--viz pcl|rerun|none] [--out <dir>] [--map-voxel-size <meters>]"
                 " [--start-time <seconds>] [--duration <seconds>]\n";
    return 1;
  }

  fastlio_app::StandaloneConfig cfg;
  try
  {
    cfg = fastlio_app::loadYamlConfig(args.configPath);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to load config '" << args.configPath << "': " << e.what() << std::endl;
    return 1;
  }

  Preprocess preprocess;
  preprocess.lidar_type = cfg.lio.lidar_type;
  preprocess.blind = cfg.blind;
  preprocess.N_SCANS = cfg.scan_line;
  preprocess.time_unit = cfg.timestamp_unit;
  preprocess.SCAN_RATE = cfg.scan_rate;
  preprocess.point_filter_num = cfg.point_filter_num;
  preprocess.feature_enabled = cfg.feature_extract_enable;

  // LioCore must be heap-allocated: it embeds a KD_TREE, whose Rebuild_Logger
  // holds a fixed ~1e6-entry buffer (tens of MB) -- overflows a thread stack.
  auto lio = std::make_unique<fastlio::LioCore>(cfg.lio);

  fastlio::PacketSync sync;
  sync.setLidarType(cfg.lio.lidar_type);

  double mapVoxelSize = args.mapVoxelSize > 0 ? args.mapVoxelSize : cfg.lio.filter_size_map_min;
  if (mapVoxelSize <= 0) mapVoxelSize = kDefaultMapVoxelSize;

  std::unique_ptr<aimcap::Reader> reader;
  std::optional<uint64_t> totalLidarScans;
  fastlio_app::BagTimeRange range;
  try
  {
    reader = std::make_unique<aimcap::Reader>(args.bagPath);
    totalLidarScans = reader->MessageCount(cfg.lid_topic);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to open bag '" << args.bagPath << "': " << e.what() << std::endl;
    return 1;
  }

  if (args.startTimeOffset > 0 || args.duration)
  {
    range.startTimeSec = args.startTimeOffset;
    if (args.duration) range.endTimeSec = args.startTimeOffset + *args.duration;
    std::cout << "Restricting replay to [" << args.startTimeOffset << "s, "
              << (args.duration ? std::to_string(args.startTimeOffset + *args.duration) : std::string("end"))
              << "s) from the bag's start" << std::endl;
    // messageCount() reflects the whole bag, not the restricted range --
    // showing it as a percentage denominator would be actively misleading.
    totalLidarScans.reset();
  }
  else if (totalLidarScans)
  {
    std::cout << "Found " << *totalLidarScans << " lidar scans on " << cfg.lid_topic << std::endl;
  }

  std::unique_ptr<fastlio::Visualizer> viz;
  if (args.viz == "none")
  {
    viz = std::make_unique<fastlio::NullVisualizer>();
  }
  else if (args.viz == "rerun")
  {
    viz = std::make_unique<fastlio::RerunVisualizer>();
  }
  else if (args.viz == "pcl")
  {
    viz = std::make_unique<fastlio::PclVisualizer>();
  }
  else
  {
    std::cerr << "Unknown --viz backend: " << args.viz << " (expected pcl, rerun, or none)" << std::endl;
    return 1;
  }

  fastlio::IncrementalVoxelMap voxelMap(mapVoxelSize);
  std::vector<TrajPoint> trajectory;
  size_t frameCount = 0;
  size_t scansSeen = 0;
  bool stopRequested = false;

  auto reportProgress = [&]() {
    if (totalLidarScans)
    {
      double pct = 100.0 * static_cast<double>(scansSeen) / static_cast<double>(*totalLidarScans);
      std::cout << "[progress] scan " << scansSeen << "/" << *totalLidarScans << " (" << std::fixed
                << std::setprecision(1) << pct << "%), frames=" << frameCount
                << ", map voxels=" << voxelMap.voxelCount() << std::endl;
    }
    else
    {
      std::cout << "[progress] scan " << scansSeen << ", frames=" << frameCount
                << ", map voxels=" << voxelMap.voxelCount() << std::endl;
    }
  };

  auto onMeasurement = [&](const MeasureGroup &meas) {
    scansSeen++;
    if (stopRequested) return;

    fastlio::LioCore::FrameResult res = lio->processFrame(meas);
    if (!res.ok)
    {
      if (scansSeen % kProgressInterval == 0) reportProgress();
      return;
    }

    PointCloudXYZI::Ptr worldCloud = toWorldFrame(res.state, res.down_body);

    viz->onPose(res.time, res.state.pos, res.state.rot);
    viz->onCloudWorld(res.time, worldCloud);
    viz->spinOnce();
    if (viz->shouldClose())
    {
      stopRequested = true;
    }

    voxelMap.addCloud(*worldCloud);
    viz->onMapUpdate(res.time, voxelMap.toCloud());
    trajectory.push_back(TrajPoint{res.time, res.state.pos, res.state.rot, res.state.vel, res.angvel});
    frameCount++;

    if (scansSeen % kProgressInterval == 0) reportProgress();

    if (cfg.runtime_pos_log_enable)
    {
      std::cout << "[frame " << frameCount << "] t=" << res.time
                << " pos=(" << res.state.pos(0) << ", " << res.state.pos(1) << ", " << res.state.pos(2) << ")"
                << " match=" << res.match_time << " solve=" << res.solve_time
                << " incr=" << res.kdtree_incremental_time << " eff_pts=" << res.effect_feat_num << std::endl;
    }
  };

  try
  {
    fastlio_app::replayBag(*reader, cfg.lid_topic, cfg.imu_topic, cfg.time_offset_lidar_to_imu,
                            preprocess, sync, onMeasurement, range);
  }
  catch (const std::exception &e)
  {
    std::cerr << "bag replay failed: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Processed " << frameCount << " frames." << std::endl;

  if (voxelMap.voxelCount() > 0)
  {
    PointCloudXYZI::Ptr finalMap = voxelMap.toCloud();
    std::string pcdPath = args.outDir + "/scans.pcd";
    pcl::io::savePCDFileBinary(pcdPath, *finalMap);
    std::cout << "Saved map (" << finalMap->size() << " points) to " << pcdPath << std::endl;
  }

  if (!trajectory.empty())
  {
    std::string trajPath = args.outDir + "/traj.tum";
    std::ofstream out(trajPath);
    out << std::fixed << std::setprecision(9);
    for (const auto &tp : trajectory)
    {
      // Standard TUM columns (timestamp tx ty tz qx qy qz qw) plus 6 extra
      // columns appended at the end -- vx vy vz (world frame, m/s) and wx wy
      // wz (bias-corrected angular velocity, IMU/body frame, rad/s) --
      // strict TUM readers (e.g. evo) expect exactly 8 whitespace-separated
      // fields per line and will reject this file.
      out << tp.time << " " << tp.pos(0) << " " << tp.pos(1) << " " << tp.pos(2) << " "
          << tp.q.x() << " " << tp.q.y() << " " << tp.q.z() << " " << tp.q.w() << " "
          << tp.vel(0) << " " << tp.vel(1) << " " << tp.vel(2) << " "
          << tp.angvel(0) << " " << tp.angvel(1) << " " << tp.angvel(2) << "\n";
    }
    std::cout << "Saved trajectory (" << trajectory.size()
              << " poses, TUM format + vx vy vz + wx wy wz columns) to " << trajPath << std::endl;
  }

  return 0;
}
