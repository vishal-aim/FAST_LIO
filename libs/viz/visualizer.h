#ifndef FASTLIO_VIZ_VISUALIZER_H
#define FASTLIO_VIZ_VISUALIZER_H

#include <Eigen/Geometry>
#include <fastlio/common_lib.h>

namespace fastlio
{

// Abstract sink for what the standalone app has to show: the growing map
// and the trajectory. Deliberately minimal so a future Rerun-backed (or any
// other) implementation is a drop-in replacement -- main.cpp and LioCore
// never need to change to swap the backend.
class Visualizer
{
 public:
  virtual ~Visualizer() = default;

  virtual void onPose(double t, const V3D &pos, const Eigen::Quaterniond &q) = 0;
  virtual void onCloudWorld(double t, PointCloudXYZI::ConstPtr cloud) = 0;

  // Pumps the backend's UI event loop; called once per processed frame.
  virtual void spinOnce() {}

  // True once the user has closed the visualization window (or an
  // equivalent backend-specific "stop" signal); the caller should end
  // playback when this becomes true.
  virtual bool shouldClose() { return false; }
};

}  // namespace fastlio

#endif
