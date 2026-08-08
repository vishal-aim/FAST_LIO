#ifndef FASTLIO_STANDALONE_BAG_SOURCE_H
#define FASTLIO_STANDALONE_BAG_SOURCE_H

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fastlio_standalone
{

// One message handed up from any container backend, already carrying its
// own encoding label so the decode layer (bag_replay.cpp) can pick the right
// wire-format reader without the container needing to know about ROS1 vs
// ROS2 message formats at all.
struct RawMessage
{
  std::string topic;
  std::string encoding;    // "ros1" or "cdr"
  double timestampSec = 0;
  const std::byte *data = nullptr;  // valid only for the duration of the forEachMessage callback
  size_t size = 0;
};

// Abstracts over the recording container (mcap, rosbag2 sqlite3/.db3, ...).
// Implementations only handle iterating their own file format in timestamp
// order and reporting each message's own declared encoding -- they know
// nothing about PointCloud2/Imu, ESIKF, or fastlio at all.
class BagSource
{
 public:
  virtual ~BagSource() = default;

  // Invokes `cb` once per message on any of `topics`, in timestamp order.
  // `cb`'s RawMessage::data pointer is only valid for the duration of that
  // single call.
  virtual void forEachMessage(const std::vector<std::string> &topics,
                               const std::function<void(const RawMessage &)> &cb) = 0;
};

// Opens the right BagSource for `path`, sniffing the container format from
// its file extension (.mcap / .db3) unless `formatOverride` forces one --
// pass "mcap" or "ros2db3" to skip sniffing. Throws std::runtime_error if
// the format can't be determined or the file can't be opened.
std::unique_ptr<BagSource> openBagSource(const std::string &path, const std::string &formatOverride = "");

}  // namespace fastlio_standalone

#endif
