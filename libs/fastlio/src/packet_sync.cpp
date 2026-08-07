#include <fastlio/packet_sync.h>

#include <fastlio/preprocess.h>

namespace fastlio
{

bool PacketSync::pushLidar(PointCloudXYZI::Ptr cloud, double beg_time)
{
  bool loop_back = beg_time < last_timestamp_lidar_;
  if (loop_back) lidar_buffer_.clear();

  lidar_buffer_.push_back(cloud);
  time_buffer_.push_back(beg_time);
  last_timestamp_lidar_ = beg_time;
  return !loop_back;
}

bool PacketSync::pushImu(const ImuSample &imu)
{
  bool loop_back = imu.timestamp < last_timestamp_imu_;
  if (loop_back) imu_buffer_.clear();

  last_timestamp_imu_ = imu.timestamp;
  imu_buffer_.push_back(imu);
  return !loop_back;
}

bool PacketSync::nextMeasurement(MeasureGroup &meas)
{
  if (lidar_buffer_.empty() || imu_buffer_.empty())
  {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed_)
  {
    meas.lidar = lidar_buffer_.front();
    meas.lidar_beg_time = time_buffer_.front();

    if (meas.lidar->points.size() <= 1) // time too little
    {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    }
    else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_)
    {
      lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
    }
    else
    {
      scan_num_ ++;
      lidar_end_time_ = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime_ += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
    }
    if (lidar_type_ == MARSIM)
      lidar_end_time_ = meas.lidar_beg_time;

    meas.lidar_end_time = lidar_end_time_;

    lidar_pushed_ = true;
  }

  if (last_timestamp_imu_ < lidar_end_time_)
  {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer_.front().timestamp;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
  {
    imu_time = imu_buffer_.front().timestamp;
    if (imu_time > lidar_end_time_) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}

} // namespace fastlio
