#ifndef FASTLIO_VIZ_NULL_VISUALIZER_H
#define FASTLIO_VIZ_NULL_VISUALIZER_H

#include "visualizer.h"

namespace fastlio
{

// --viz none: run the algorithm and still write PCD/trajectory output, just
// without a visualization backend.
class NullVisualizer : public Visualizer
{
 public:
  void onPose(double, const V3D &, const Eigen::Quaterniond &) override {}
  void onCloudWorld(double, PointCloudXYZI::ConstPtr) override {}
};

}  // namespace fastlio

#endif
