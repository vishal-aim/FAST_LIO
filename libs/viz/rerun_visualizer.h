#ifndef FASTLIO_VIZ_RERUN_VISUALIZER_H
#define FASTLIO_VIZ_RERUN_VISUALIZER_H

#include <rerun.hpp>
#include "visualizer.h"

namespace fastlio
{

// Visualizer backed by the Rerun SDK (https://rerun.io). Spawns (or attaches
// to) the Rerun Viewer and streams data to it over gRPC rather than
// rendering locally -- there's no local event loop to pump or window to
// close, so spinOnce()/shouldClose() stay at Visualizer's defaults.
class RerunVisualizer : public Visualizer
{
 public:
  RerunVisualizer();

  void onPose(double t, const V3D &pos, const Eigen::Quaterniond &q) override;
  void onCloudWorld(double t, PointCloudXYZI::ConstPtr cloud) override;
  void onMapUpdate(double t, PointCloudXYZI::ConstPtr mapCloud) override;

 private:
  rerun::RecordingStream rec_;
  std::vector<rerun::datatypes::Vec3D> trajectory_;
};

}  // namespace fastlio

#endif
