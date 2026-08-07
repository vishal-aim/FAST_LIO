#ifndef FASTLIO_STANDALONE_VIZ_PCL_VISUALIZER_H
#define FASTLIO_STANDALONE_VIZ_PCL_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "visualizer.h"

namespace fastlio_standalone
{

// Visualizer backed by pcl::visualization::PCLVisualizer -- already a linked
// dependency via PCL, so this adds no new third-party library.
class PclVisualizer : public Visualizer
{
 public:
  PclVisualizer();

  void onPose(double t, const V3D &pos, const Eigen::Quaterniond &q) override;
  void onCloudWorld(double t, PointCloudXYZI::ConstPtr cloud) override;
  void spinOnce() override;
  bool shouldClose() override;

 private:
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  PointCloudXYZI::Ptr map_;
  bool haveLastPos_ = false;
  V3D lastPos_ = V3D::Zero();
  int lineId_ = 0;
  int frameCount_ = 0;
};

}  // namespace fastlio_standalone

#endif
