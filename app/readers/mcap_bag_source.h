#ifndef FASTLIO_APP_READERS_MCAP_BAG_SOURCE_H
#define FASTLIO_APP_READERS_MCAP_BAG_SOURCE_H

#include <string>
#include "bag_source.h"

namespace fastlio_app
{

// BagSource backed by an mcap file. Iterates in log-time order and reports
// each channel's own messageEncoding ("ros1" or "cdr", depending on whether
// the recording originated from ROS1 or ROS2) -- decoding is entirely the
// caller's concern via bag_replay.cpp.
class McapBagSource : public BagSource
{
 public:
  explicit McapBagSource(const std::string &path);

  void forEachMessage(const std::vector<std::string> &topics,
                       const std::function<void(const RawMessage &)> &cb,
                       const BagTimeRange &range = {}) override;

  std::optional<size_t> messageCount(const std::string &topic) override;
  std::optional<double> firstMessageTime(const std::vector<std::string> &topics) override;

 private:
  std::string path_;
};

}  // namespace fastlio_app

#endif
