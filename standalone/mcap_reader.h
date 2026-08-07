#ifndef FASTLIO_STANDALONE_MCAP_READER_H
#define FASTLIO_STANDALONE_MCAP_READER_H

#include <functional>
#include <string>
#include <fastlio/common_lib.h>
#include <fastlio/packet_sync.h>
#include <fastlio/preprocess.h>

namespace fastlio_standalone
{

// Replays an mcap recording's lidar (sensor_msgs/PointCloud2) and IMU
// (sensor_msgs/Imu) topics through `preprocess`/`sync` in log-time order,
// invoking `onMeasurement` for each MeasureGroup PacketSync produces --
// mirroring what the ROS1 node's callbacks + main loop do for a live stream,
// just synchronous and single-threaded since there's no realtime source here.
//
// Only "ros1"-encoded channels (schema encoding "ros1msg") are supported --
// this targets bags recorded/converted from ROS1, matching the rest of this
// project. A ROS2 (cdr-encoded) recording will fail with a clear error
// rather than silently misdecoding.
//
// The PointCloud2 sensor layout (Velodyne "ring"+"time" vs Ouster "t"+
// "reflectivity"+"ring"+"ambient"+"range" vs a generic XYZI cloud) is
// detected from the message's own `fields[]` array, not hardcoded, and
// dispatched to the matching fastlio::Preprocess::process() overload.
void replayMcap(const std::string &mcapPath,
                 const std::string &lidTopic,
                 const std::string &imuTopic,
                 double timeOffsetLidarToImu,
                 Preprocess &preprocess,
                 fastlio::PacketSync &sync,
                 const std::function<void(const MeasureGroup &)> &onMeasurement);

}  // namespace fastlio_standalone

#endif
