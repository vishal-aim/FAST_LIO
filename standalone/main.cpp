#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <pcl/io/pcd_io.h>

#include <fastlio/lio_core.h>
#include <fastlio/packet_sync.h>
#include <fastlio/preprocess.h>

#include "bag_replay.h"
#include "bag_source.h"
#include "config.h"
#include "viz/null_visualizer.h"
#include "viz/pcl_visualizer.h"
#include "viz/visualizer.h"

namespace
{

struct Args
{
  std::string bagPath;
  std::string configPath;
  std::string outDir = ".";
  std::string format;  // "" (auto-detect from extension), "mcap", or "ros2db3"
  bool headless = false;
};

struct TrajPoint
{
  double time;
  V3D pos;
  Eigen::Quaterniond q;
};

bool parseArgs(int argc, char **argv, Args &args)
{
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    if (arg == "--bag" && i + 1 < argc) args.bagPath = argv[++i];
    else if (arg == "--config" && i + 1 < argc) args.configPath = argv[++i];
    else if (arg == "--out" && i + 1 < argc) args.outDir = argv[++i];
    else if (arg == "--format" && i + 1 < argc) args.format = argv[++i];
    else if (arg == "--headless") args.headless = true;
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
              << " --bag <file.mcap|file.db3> --config <config.yaml> [--format mcap|ros2db3]"
                 " [--headless] [--out <dir>]\n";
    return 1;
  }

  fastlio_standalone::StandaloneConfig cfg;
  try
  {
    cfg = fastlio_standalone::loadYamlConfig(args.configPath);
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

  std::unique_ptr<fastlio_standalone::Visualizer> viz;
  if (args.headless)
  {
    viz = std::make_unique<fastlio_standalone::NullVisualizer>();
  }
  else
  {
    viz = std::make_unique<fastlio_standalone::PclVisualizer>();
  }

  PointCloudXYZI::Ptr mapAccum(new PointCloudXYZI());
  std::vector<TrajPoint> trajectory;
  size_t frameCount = 0;
  bool stopRequested = false;

  auto onMeasurement = [&](const MeasureGroup &meas) {
    if (stopRequested) return;

    fastlio::LioCore::FrameResult res = lio->processFrame(meas);
    if (!res.ok) return;

    PointCloudXYZI::Ptr worldCloud = toWorldFrame(res.state, res.down_body);

    viz->onPose(res.time, res.state.pos, res.state.rot);
    viz->onCloudWorld(res.time, worldCloud);
    viz->spinOnce();
    if (viz->shouldClose())
    {
      stopRequested = true;
    }

    *mapAccum += *worldCloud;
    trajectory.push_back(TrajPoint{res.time, res.state.pos, res.state.rot});
    frameCount++;

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
    auto source = fastlio_standalone::openBagSource(args.bagPath, args.format);
    fastlio_standalone::replayBag(*source, cfg.lid_topic, cfg.imu_topic, cfg.time_offset_lidar_to_imu,
                                   preprocess, sync, onMeasurement);
  }
  catch (const std::exception &e)
  {
    std::cerr << "bag replay failed: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Processed " << frameCount << " frames." << std::endl;

  if (!mapAccum->empty())
  {
    std::string pcdPath = args.outDir + "/scans.pcd";
    pcl::io::savePCDFileBinary(pcdPath, *mapAccum);
    std::cout << "Saved map (" << mapAccum->size() << " points) to " << pcdPath << std::endl;
  }

  if (!trajectory.empty())
  {
    std::string trajPath = args.outDir + "/traj.tum";
    std::ofstream out(trajPath);
    out << std::fixed << std::setprecision(9);
    for (const auto &tp : trajectory)
    {
      out << tp.time << " " << tp.pos(0) << " " << tp.pos(1) << " " << tp.pos(2) << " "
          << tp.q.x() << " " << tp.q.y() << " " << tp.q.z() << " " << tp.q.w() << "\n";
    }
    std::cout << "Saved trajectory (" << trajectory.size() << " poses, TUM format) to " << trajPath << std::endl;
  }

  return 0;
}
