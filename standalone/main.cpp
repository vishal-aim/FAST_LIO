#include <iostream>
#include <memory>
#include <fastlio/lio_core.h>
#include <fastlio/packet_sync.h>

int main()
{
  fastlio::LioConfig cfg;
  // LioCore must be heap-allocated: it embeds a KD_TREE, whose Rebuild_Logger
  // holds a fixed ~1e6-entry buffer (tens of MB) -- fine as a global (the
  // original ROS node's storage) or on the heap, but stack-allocating LioCore
  // overflows the default thread stack.
  auto lio = std::make_unique<fastlio::LioCore>(cfg);
  fastlio::PacketSync sync;
  std::cout << "fastlio smoke build OK, map initialized: " << lio->mapInitialized() << std::endl;
  return 0;
}
