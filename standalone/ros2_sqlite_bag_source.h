#ifndef FASTLIO_STANDALONE_ROS2_SQLITE_BAG_SOURCE_H
#define FASTLIO_STANDALONE_ROS2_SQLITE_BAG_SOURCE_H

#include <string>
#include "bag_source.h"

namespace fastlio_standalone
{

// BagSource backed by a rosbag2 sqlite3 (.db3) file: a plain SQLite database
// with a `topics(id, name, type, serialization_format, ...)` table and a
// `messages(topic_id, timestamp, data)` table, the latter already indexed by
// timestamp. Reads only the single .db3 file passed in -- multi-file bags
// (several .db3 shards listed in a metadata.yaml) aren't handled, since
// every recording seen so far has been a single shard; revisit if that
// changes. `messages.timestamp` is nanoseconds since epoch; the encoding
// reported per message is whatever `serialization_format` says (normally
// "cdr" -- decode dispatch and any other value is rejected in bag_replay.cpp,
// not here).
class Ros2SqliteBagSource : public BagSource
{
 public:
  explicit Ros2SqliteBagSource(const std::string &path);

  void forEachMessage(const std::vector<std::string> &topics,
                       const std::function<void(const RawMessage &)> &cb,
                       const BagTimeRange &range = {}) override;

  std::optional<size_t> messageCount(const std::string &topic) override;
  std::optional<double> firstMessageTime(const std::vector<std::string> &topics) override;

 private:
  std::string path_;
};

}  // namespace fastlio_standalone

#endif
