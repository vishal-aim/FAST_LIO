#ifndef FASTLIO_PACKET_SYNC_H
#define FASTLIO_PACKET_SYNC_H

#include <deque>
#include <fastlio/common_lib.h>
#include <fastlio/types.h>

namespace fastlio
{

// Buffers lidar scans and IMU samples and groups them into MeasureGroups,
// exactly mirroring the original ROS node's sync_packages()/standard_pcl_cbk/
// imu_cbk buffering logic -- just without the ROS callback threading (a
// single-threaded mcap replay can call pushLidar/pushImu/nextMeasurement
// directly; a ROS node should still guard calls with its own mutex since its
// callbacks run on a different thread than the processing loop).
class PacketSync
{
 public:
  void setLidarType(int lidar_type) { lidar_type_ = lidar_type; }

  // Returns false if beg_time went backwards relative to the previously
  // pushed scan -- the buffer is cleared either way (matches the original
  // "lidar loop back, clear buffer" recovery), the return value just lets
  // the caller decide whether to log it.
  bool pushLidar(PointCloudXYZI::Ptr cloud, double beg_time);

  // Same loop-back semantics as pushLidar, for the IMU stream.
  bool pushImu(const ImuSample &imu);

  // Pops one synchronized MeasureGroup once enough IMU data has arrived to
  // cover the oldest buffered lidar scan. Returns false if not ready yet.
  bool nextMeasurement(MeasureGroup &meas);

  // Timestamp of the most recently pushed sample of each stream -- e.g. used
  // by a Livox-specific IMU/lidar self-sync heuristic upstream of this class.
  double lastLidarTime() const { return last_timestamp_lidar_; }
  double lastImuTime() const { return last_timestamp_imu_; }

 private:
  int lidar_type_ = 1; // AVIA, see preprocess.h::LID_TYPE

  std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> time_buffer_;
  std::deque<ImuSample> imu_buffer_;

  bool lidar_pushed_ = false;
  double lidar_end_time_ = 0.0;
  double lidar_mean_scantime_ = 0.0;
  int scan_num_ = 0;

  double last_timestamp_lidar_ = 0.0;
  double last_timestamp_imu_ = -1.0;
};

} // namespace fastlio

#endif
