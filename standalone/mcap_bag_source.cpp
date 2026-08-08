#include "mcap_bag_source.h"

#include <iostream>
#include <stdexcept>
#include <mcap/reader.hpp>

namespace fastlio_standalone
{

// The mcap::McapReader itself is opened lazily in forEachMessage() rather
// than stored as a member, since keeping it out of this header avoids
// pulling <mcap/reader.hpp> (and its C++17 requirement) into every
// translation unit that just wants to construct a McapBagSource.
McapBagSource::McapBagSource(const std::string &path) : path_(path) {}

void McapBagSource::forEachMessage(const std::vector<std::string> &topics,
                                    const std::function<void(const RawMessage &)> &cb,
                                    const BagTimeRange &range)
{
  mcap::McapReader reader;
  {
    const mcap::Status status = reader.open(path_);
    if (!status.ok())
    {
      throw std::runtime_error("failed to open mcap file '" + path_ + "': " + status.message);
    }
  }

  mcap::ReadMessageOptions options;
  options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
  if (range.startTimeSec) options.startTime = static_cast<mcap::Timestamp>(*range.startTimeSec * 1e9);
  if (range.endTimeSec) options.endTime = static_cast<mcap::Timestamp>(*range.endTimeSec * 1e9);
  options.topicFilter = [&](std::string_view topic) {
    for (const auto &t : topics)
    {
      if (topic == t) return true;
    }
    return false;
  };

  auto onProblem = [](const mcap::Status &problem) {
    std::cerr << "[mcap] " << problem.message << std::endl;
  };

  for (const auto &view : reader.readMessages(onProblem, options))
  {
    RawMessage msg;
    msg.topic = view.channel->topic;
    msg.encoding = view.channel->messageEncoding;
    msg.timestampSec = static_cast<double>(view.message.logTime) * 1e-9;
    msg.data = view.message.data;
    msg.size = view.message.dataSize;
    cb(msg);
  }
}

std::optional<size_t> McapBagSource::messageCount(const std::string &topic)
{
  mcap::McapReader reader;
  if (!reader.open(path_).ok()) return std::nullopt;
  if (!reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan).ok()) return std::nullopt;

  const auto &stats = reader.statistics();
  if (!stats) return std::nullopt;

  for (const auto &[channelId, channelPtr] : reader.channels())
  {
    if (channelPtr && channelPtr->topic == topic)
    {
      auto it = stats->channelMessageCounts.find(channelId);
      if (it != stats->channelMessageCounts.end()) return static_cast<size_t>(it->second);
    }
  }
  return std::nullopt;
}

std::optional<double> McapBagSource::firstMessageTime(const std::vector<std::string> & /*topics*/)
{
  // mcap's Statistics only tracks start/end time across the whole file, not
  // per-channel -- close enough for turning a relative --duration into an
  // absolute range, since a recording's topics all start around the same
  // time in practice.
  mcap::McapReader reader;
  if (!reader.open(path_).ok()) return std::nullopt;
  if (!reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan).ok()) return std::nullopt;

  const auto &stats = reader.statistics();
  if (!stats) return std::nullopt;
  return static_cast<double>(stats->messageStartTime) * 1e-9;
}

}  // namespace fastlio_standalone
