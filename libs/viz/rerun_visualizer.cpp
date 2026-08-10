#include "rerun_visualizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fastlio
{

namespace
{

// Constant, not intensity-derived: raw LiDAR intensity ranges vary wildly by
// sensor/driver (0-1, 0-255, 0-65535, ...), so a fixed shade per entity is
// more legible than a naive per-point mapping that ends up all-black or
// all-white depending on the source.
const rerun::components::Color kScanColor(200, 200, 200);  // light gray

// Half Rerun's own default Points3D radius (1.5 ui points) -- ui points
// rather than scene units so point size stays constant on screen regardless
// of zoom, matching the auto-sized default this replaces.
const rerun::components::Radius kPointRadius = rerun::components::Radius::ui_points(0.75f);

// Visual length (meters) of the RGB axis arrows drawn for the world origin
// and the robot's current pose.
constexpr float kAxisLength = 1.0f;

std::vector<rerun::components::Position3D> toPositions(PointCloudXYZI::ConstPtr cloud)
{
  std::vector<rerun::components::Position3D> positions;
  positions.reserve(cloud->size());
  for (const auto &p : cloud->points) positions.emplace_back(p.x, p.y, p.z);
  return positions;
}

// Blue (low) -> green -> red (high): hue swept from 240deg down to 0deg at
// full saturation/value, t clamped to [0, 1].
rerun::components::Color heightToColor(float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  float hue = (1.0f - t) * 240.0f;
  float x = 1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f);
  float r, g, b;
  if (hue < 60) { r = 1; g = x; b = 0; }
  else if (hue < 120) { r = x; g = 1; b = 0; }
  else if (hue < 180) { r = 0; g = 1; b = x; }
  else { r = 0; g = x; b = 1; }
  return rerun::components::Color(
      static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255), static_cast<uint8_t>(b * 255));
}

// Map points colored by height (world-frame z): each voxel's centroid gets a
// color relative to the map's own current z range, so the gradient always
// spans the full visible height regardless of the environment's absolute
// altitude.
void toPositionsColoredByHeight(
    PointCloudXYZI::ConstPtr cloud,
    std::vector<rerun::components::Position3D> &positions,
    std::vector<rerun::components::Color> &colors)
{
  positions.reserve(cloud->size());
  colors.reserve(cloud->size());

  float minZ = std::numeric_limits<float>::max();
  float maxZ = std::numeric_limits<float>::lowest();
  for (const auto &p : cloud->points)
  {
    minZ = std::min(minZ, p.z);
    maxZ = std::max(maxZ, p.z);
  }

  for (const auto &p : cloud->points)
  {
    positions.emplace_back(p.x, p.y, p.z);
    float t = (maxZ > minZ) ? (p.z - minZ) / (maxZ - minZ) : 0.5f;
    colors.push_back(heightToColor(t));
  }
}

}  // namespace

RerunVisualizer::RerunVisualizer() : rec_("fastlio")
{
  rec_.spawn().exit_on_failure();
  rec_.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  rec_.log_static("world/origin", rerun::Transform3D(), rerun::TransformAxes3D(kAxisLength));
}

void RerunVisualizer::onPose(double t, const V3D &pos, const Eigen::Quaterniond &q)
{
  rec_.set_time_duration_secs("bag_time", t);

  rec_.log(
      "world/robot",
      rerun::Transform3D::from_translation_rotation(
          rerun::components::Translation3D(
              static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(pos.z())),
          rerun::datatypes::Quaternion::from_xyzw(
              static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()),
              static_cast<float>(q.w()))),
      rerun::TransformAxes3D(kAxisLength));

  trajectory_.emplace_back(static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(pos.z()));
  rec_.log("world/trajectory", rerun::LineStrips3D(rerun::components::LineStrip3D(trajectory_)));
}

void RerunVisualizer::onCloudWorld(double t, PointCloudXYZI::ConstPtr cloud)
{
  rec_.set_time_duration_secs("bag_time", t);
  rec_.log("world/scan", rerun::Points3D(toPositions(cloud)).with_colors(kScanColor).with_radii(kPointRadius));
}

void RerunVisualizer::onMapUpdate(double t, PointCloudXYZI::ConstPtr mapCloud)
{
  rec_.set_time_duration_secs("bag_time", t);

  std::vector<rerun::components::Position3D> positions;
  std::vector<rerun::components::Color> colors;
  toPositionsColoredByHeight(mapCloud, positions, colors);
  rec_.log("world/map", rerun::Points3D(positions).with_colors(colors).with_radii(kPointRadius));
}

}  // namespace fastlio
