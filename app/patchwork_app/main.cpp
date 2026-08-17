#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rerun.hpp>

#include <aimcap/reader.hpp>
#include <patchwork/patchworkpp.h>

#include "readers/point_field_reader.h"

namespace
{

// Constant colors (not intensity-derived, see rerun_visualizer.cpp for the
// same reasoning) so ground/non-ground read clearly regardless of the
// source sensor's raw intensity range.
const rerun::components::Color kGroundColor(60, 200, 60);     // green
const rerun::components::Color kNongroundColor(220, 60, 60);  // red
const rerun::components::Radius kPointRadius = rerun::components::Radius::ui_points(0.75f);

struct Args
{
  std::string bagPath;  // a single .mcap file, or a directory of numbered chunks
  std::string topic = "/ouster/points";
  double sensorHeight = -1.0;       // <=0: keep patchwork::Params' own default (1.723m), unless estimated below
  bool estimateSensorHeight = false;
  double estimateHeightMinRadius = 1.0;  // meters -- excludes the sensor's own mount/blind zone
  double estimateHeightMaxRadius = 3.0;  // meters -- close enough that the ground still dominates returns
  double startTimeOffset = 0.0;     // seconds from the bag's first message on `topic`
  std::optional<double> duration;   // seconds to process, from startTimeOffset
};

bool parseArgs(int argc, char **argv, Args &args)
{
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    if (arg == "--bag" && i + 1 < argc) args.bagPath = argv[++i];
    else if (arg == "--topic" && i + 1 < argc) args.topic = argv[++i];
    else if (arg == "--sensor-height" && i + 1 < argc) args.sensorHeight = std::stod(argv[++i]);
    else if (arg == "--estimate-sensor-height") args.estimateSensorHeight = true;
    else if (arg == "--estimate-height-min-radius" && i + 1 < argc) args.estimateHeightMinRadius = std::stod(argv[++i]);
    else if (arg == "--estimate-height-max-radius" && i + 1 < argc) args.estimateHeightMaxRadius = std::stod(argv[++i]);
    else if (arg == "--start-time" && i + 1 < argc) args.startTimeOffset = std::stod(argv[++i]);
    else if (arg == "--duration" && i + 1 < argc) args.duration = std::stod(argv[++i]);
    else
    {
      std::cerr << "Unknown argument: " << arg << std::endl;
      return false;
    }
  }
  return !args.bagPath.empty();
}

// Estimates height above ground from the sensor's own first scan: within a
// ring close enough to the sensor that the ground still dominates returns
// (but outside its own mount/blind zone), most points should be ground, so
// their median z in the sensor frame approximates -sensor_height. Assumes
// the sensor frame is upright (z up) and the ground nearby is roughly flat
// -- same assumption Patchwork++ itself makes via params.sensor_height.
// Returns std::nullopt if no points fall in [minRadius, maxRadius].
std::optional<double> estimateSensorHeight(const Eigen::MatrixXf &cloud, double minRadius, double maxRadius)
{
  std::vector<float> nearZs;
  for (int i = 0; i < cloud.rows(); i++)
  {
    float x = cloud(i, 0), y = cloud(i, 1), z = cloud(i, 2);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    float r = std::hypot(x, y);
    if (r >= minRadius && r <= maxRadius) nearZs.push_back(z);
  }
  if (nearZs.empty()) return std::nullopt;

  auto mid = nearZs.begin() + nearZs.size() / 2;
  std::nth_element(nearZs.begin(), mid, nearZs.end());
  return -static_cast<double>(*mid);
}

// Patchwork++'s own estimateGround() contract: an Nx4 (x, y, z, intensity)
// matrix in the sensor's own frame -- no pcl::PointCloud round-trip needed,
// unlike fastlio_app's decode path. Ground segmentation deliberately stays
// self-contained rather than growing readers/ or bag_replay.cpp -- only the
// generic field-byte-reading helper (point_field_reader.h) is shared.
Eigen::MatrixXf toPatchworkCloud(const aimcap::DecodedPointCloud &pc)
{
  const auto *fx = fastlio_app::findField(pc.fields, "x");
  const auto *fy = fastlio_app::findField(pc.fields, "y");
  const auto *fz = fastlio_app::findField(pc.fields, "z");
  const auto *fi = fastlio_app::findField(pc.fields, "intensity");
  if (!fx || !fy || !fz)
  {
    throw std::runtime_error("PointCloud missing a required field (x/y/z)");
  }

  size_t n = pc.point_step == 0 ? 0 : pc.data.size() / pc.point_step;
  Eigen::MatrixXf cloud(n, 4);
  for (size_t i = 0; i < n; i++)
  {
    const uint8_t *base = pc.data.data() + i * pc.point_step;
    cloud(i, 0) = static_cast<float>(fastlio_app::readFieldAsDouble(base, *fx));
    cloud(i, 1) = static_cast<float>(fastlio_app::readFieldAsDouble(base, *fy));
    cloud(i, 2) = static_cast<float>(fastlio_app::readFieldAsDouble(base, *fz));
    cloud(i, 3) = fi ? static_cast<float>(fastlio_app::readFieldAsDouble(base, *fi)) : 0.f;
  }
  return cloud;
}

std::vector<rerun::components::Position3D> toPositions(const Eigen::MatrixX3f &cloud)
{
  std::vector<rerun::components::Position3D> positions;
  positions.reserve(cloud.rows());
  for (int i = 0; i < cloud.rows(); i++) positions.emplace_back(cloud(i, 0), cloud(i, 1), cloud(i, 2));
  return positions;
}

}  // namespace

int main(int argc, char **argv)
{
  Args args;
  if (!parseArgs(argc, argv, args))
  {
    std::cerr << "Usage: " << argv[0]
              << " --bag <file.mcap|chunk_dir> [--topic /ouster/points]"
                 " [--sensor-height <meters> | --estimate-sensor-height"
                 " [--estimate-height-min-radius <m>] [--estimate-height-max-radius <m>]]"
                 " [--start-time <seconds>] [--duration <seconds>]\n";
    return 1;
  }

  patchwork::Params params;
  if (args.sensorHeight > 0) params.sensor_height = args.sensorHeight;
  // Constructed lazily on the first decoded scan below -- estimateSensorHeight()
  // (if requested) needs that scan's points first, and PatchWorkpp has no
  // setter for sensor_height after construction.
  std::unique_ptr<patchwork::PatchWorkpp> patchwork;

  std::unique_ptr<aimcap::Reader> reader;
  try
  {
    reader = std::make_unique<aimcap::Reader>(args.bagPath);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failed to open bag '" << args.bagPath << "': " << e.what() << std::endl;
    return 1;
  }

  rerun::RecordingStream rec("patchwork_app");
  rec.spawn().exit_on_failure();
  rec.log_static("sensor", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

  size_t scanCount = 0;
  // aimcap::Reader has no "restrict to a time range" replay mode -- the
  // topic's own first message becomes the reference point for
  // --start-time/--duration, same approach as fastlio_app/bag_replay.cpp.
  std::optional<double> bagStartSec;
  reader->OnPointCloud(
      args.topic, [&](const std::string & /*topic*/, const aimcap::DecodedPointCloud &pc) {
        double timestampSec = static_cast<double>(pc.stamp_sec) + static_cast<double>(pc.stamp_nsec) * 1e-9;
        if (!bagStartSec) bagStartSec = timestampSec;
        double rel = timestampSec - *bagStartSec;
        if (rel < args.startTimeOffset) return;
        if (args.duration && rel >= args.startTimeOffset + *args.duration) return;

        Eigen::MatrixXf cloud = toPatchworkCloud(pc);

        if (!patchwork)
        {
          if (args.estimateSensorHeight)
          {
            std::optional<double> estimated =
                estimateSensorHeight(cloud, args.estimateHeightMinRadius, args.estimateHeightMaxRadius);
            if (estimated)
            {
              params.sensor_height = *estimated;
              std::cout << "Estimated sensor height: " << *estimated << "m (median z within ["
                        << args.estimateHeightMinRadius << ", " << args.estimateHeightMaxRadius
                        << "]m of the sensor)" << std::endl;
            }
            else
            {
              std::cerr << "warning: no points within [" << args.estimateHeightMinRadius << ", "
                         << args.estimateHeightMaxRadius << "]m to estimate sensor height from, using "
                        << params.sensor_height << "m" << std::endl;
            }
          }
          patchwork = std::make_unique<patchwork::PatchWorkpp>(params);
        }

        patchwork->estimateGround(cloud);

        Eigen::MatrixX3f ground = patchwork->getGround();
        Eigen::MatrixX3f nonground = patchwork->getNonground();

        rec.set_time_duration_secs("bag_time", timestampSec);
        rec.log("sensor/ground",
                rerun::Points3D(toPositions(ground)).with_colors(kGroundColor).with_radii(kPointRadius));
        rec.log("sensor/nonground",
                rerun::Points3D(toPositions(nonground)).with_colors(kNongroundColor).with_radii(kPointRadius));

        scanCount++;
        std::cout << "[scan " << scanCount << "] points=" << cloud.rows() << " ground=" << ground.rows()
                  << " nonground=" << nonground.rows() << " time=" << (patchwork->getTimeTaken() / 1000.0) << "ms"
                  << std::endl;
      });

  try
  {
    reader->Run();
  }
  catch (const std::exception &e)
  {
    std::cerr << "bag replay failed: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Processed " << scanCount << " scans." << std::endl;
  if (scanCount == 0)
  {
    std::cerr << "warning: no messages found on topic '" << args.topic << "'" << std::endl;
  }

  return 0;
}
