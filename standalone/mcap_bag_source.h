#ifndef FASTLIO_STANDALONE_MCAP_BAG_SOURCE_H
#define FASTLIO_STANDALONE_MCAP_BAG_SOURCE_H

#include <string>
#include "bag_source.h"

namespace fastlio_standalone
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
                       const std::function<void(const RawMessage &)> &cb) override;

  std::optional<size_t> messageCount(const std::string &topic) override;

 private:
  std::string path_;
};

}  // namespace fastlio_standalone

#endif
