#include "pcl_visualizer.h"

namespace fastlio_standalone
{

PclVisualizer::PclVisualizer()
    : viewer_(new pcl::visualization::PCLVisualizer("FAST-LIO (standalone)")),
      map_(new PointCloudXYZI())
{
  viewer_->setBackgroundColor(0, 0, 0);
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();

  pcl::visualization::PointCloudColorHandlerGenericField<PointType> colorHandler(map_, "intensity");
  viewer_->addPointCloud<PointType>(map_, colorHandler, "map");
  viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
}

void PclVisualizer::onPose(double /*t*/, const V3D &pos, const Eigen::Quaterniond & /*q*/)
{
  pcl::PointXYZ p(static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(pos.z()));

  if (haveLastPos_)
  {
    pcl::PointXYZ prev(static_cast<float>(lastPos_.x()), static_cast<float>(lastPos_.y()), static_cast<float>(lastPos_.z()));
    viewer_->addLine(prev, p, 1.0, 0.0, 0.0, "traj_" + std::to_string(lineId_++));
  }
  lastPos_ = pos;
  haveLastPos_ = true;

  Eigen::Affine3f camPose = Eigen::Affine3f::Identity();
  camPose.translation() = pos.cast<float>();
  viewer_->setCameraPosition(pos.x() - 20, pos.y() - 20, pos.z() + 20, pos.x(), pos.y(), pos.z(), 0, 0, 1);
}

void PclVisualizer::onCloudWorld(double /*t*/, PointCloudXYZI::ConstPtr cloud)
{
  *map_ += *cloud;
  frameCount_++;

  pcl::visualization::PointCloudColorHandlerGenericField<PointType> colorHandler(map_, "intensity");
  viewer_->updatePointCloud<PointType>(map_, colorHandler, "map");
}

void PclVisualizer::spinOnce()
{
  viewer_->spinOnce(1);
}

bool PclVisualizer::shouldClose()
{
  return viewer_->wasStopped();
}

}  // namespace fastlio_standalone
