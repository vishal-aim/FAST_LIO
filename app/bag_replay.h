#ifndef FASTLIO_APP_BAG_REPLAY_H
#define FASTLIO_APP_BAG_REPLAY_H

#include <functional>
#include <string>
#include <fastlio/common_lib.h>
#include <fastlio/packet_sync.h>
#include <fastlio/preprocess.h>
#include "readers/bag_source.h"

namespace fastlio_app
{

// Replays `source`'s lidar (sensor_msgs/PointCloud2) and IMU (sensor_msgs/
// Imu) topics through `preprocess`/`sync` in timestamp order, invoking
// `onMeasurement` for each MeasureGroup PacketSync produces -- mirroring
// what the ROS1 node's callbacks + main loop do for a live stream, just
// synchronous and single-threaded since there's no realtime source here.
// Works with any BagSource (mcap, rosbag2 sqlite3, ...); each message's own
// declared encoding ("ros1" or "cdr") picks the matching wire-format decoder
// -- ros1_deserialize.h or ros2_cdr_deserialize.h -- so a single mcap file
// or a single .db3 can in principle mix ROS1- and ROS2-sourced topics.
//
// The PointCloud2 sensor layout (Velodyne "ring"+"time" vs Ouster "t"+
// "reflectivity"+"ring"+"ambient"+"range" vs a generic XYZI cloud) is
// detected from the message's own `fields[]` array, not hardcoded, and
// dispatched to the matching fastlio::Preprocess::process() overload.
void replayBag(BagSource &source,
                const std::string &lidTopic,
                const std::string &imuTopic,
                double timeOffsetLidarToImu,
                Preprocess &preprocess,
                fastlio::PacketSync &sync,
                const std::function<void(const MeasureGroup &)> &onMeasurement,
                const BagTimeRange &range = {});

}  // namespace fastlio_app

#endif
