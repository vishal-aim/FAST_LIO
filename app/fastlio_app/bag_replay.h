#ifndef FASTLIO_APP_BAG_REPLAY_H
#define FASTLIO_APP_BAG_REPLAY_H

#include <functional>
#include <optional>
#include <string>

#include <aimcap/reader.hpp>
#include <fastlio/common_lib.h>
#include <fastlio/packet_sync.h>
#include <fastlio/preprocess.h>

namespace fastlio_app
{

// Absolute epoch-second bounds for a replayBag() call, relative to the
// bag's own first lidar-topic timestamp. Both optional -- unset means
// unbounded. Filtering happens client-side (aimcap::Reader::Run() always
// decodes every subscribed topic across all input files) rather than via a
// container-level seek.
struct BagTimeRange
{
  std::optional<double> startTimeSec;
  std::optional<double> endTimeSec;
};

// Replays `reader`'s lidar (point cloud) and IMU topics through
// `preprocess`/`sync` in per-file log-time order, invoking `onMeasurement`
// for each MeasureGroup PacketSync produces -- mirroring what the ROS1
// node's callbacks + main loop do for a live stream, just synchronous and
// single-threaded since there's no realtime source here. `reader` may be
// backed by a single .mcap file or a directory of numbered chunks (raw,
// aimcap-concat'd, or aimcap-compressed) -- aimcap::Reader itself decodes
// whichever of the point cloud schemas it actually is on disk.
//
// The PointCloud2 sensor layout (Velodyne "ring"+"time" vs Ouster "t"+
// "reflectivity"+"ring"+"ambient"+"range" vs a generic XYZI cloud) is
// detected from the decoded cloud's own `fields[]` array, not hardcoded, and
// dispatched to the matching fastlio::Preprocess::process() overload.
void replayBag(aimcap::Reader &reader,
                const std::string &lidTopic,
                const std::string &imuTopic,
                double timeOffsetLidarToImu,
                Preprocess &preprocess,
                fastlio::PacketSync &sync,
                const std::function<void(const MeasureGroup &)> &onMeasurement,
                const BagTimeRange &range = {});

}  // namespace fastlio_app

#endif
