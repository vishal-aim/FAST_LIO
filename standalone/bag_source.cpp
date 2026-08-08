#include "bag_source.h"

#include <algorithm>
#include <stdexcept>

#include "mcap_bag_source.h"
#include "ros2_sqlite_bag_source.h"

namespace fastlio_standalone
{

namespace
{

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

bool hasExtension(const std::string &path, const std::string &ext)
{
  return path.size() >= ext.size() && toLower(path.substr(path.size() - ext.size())) == ext;
}

}  // namespace

std::unique_ptr<BagSource> openBagSource(const std::string &path, const std::string &formatOverride)
{
  std::string format = formatOverride;
  if (format.empty())
  {
    if (hasExtension(path, ".mcap")) format = "mcap";
    else if (hasExtension(path, ".db3")) format = "ros2db3";
    else
    {
      throw std::runtime_error("could not detect bag format for '" + path +
                                "' from its extension (expected .mcap or .db3) -- pass --format explicitly");
    }
  }

  if (format == "mcap") return std::make_unique<McapBagSource>(path);
  if (format == "ros2db3") return std::make_unique<Ros2SqliteBagSource>(path);

  throw std::runtime_error("unknown bag format '" + format + "' (expected 'mcap' or 'ros2db3')");
}

}  // namespace fastlio_standalone
