#ifndef FASTLIO_STANDALONE_INCREMENTAL_VOXEL_MAP_H
#define FASTLIO_STANDALONE_INCREMENTAL_VOXEL_MAP_H

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <fastlio/common_lib.h>

namespace fastlio_standalone
{

// A running per-voxel centroid, updated incrementally as points arrive --
// unlike periodically re-running pcl::VoxelGrid over the whole accumulated
// cloud, this touches each incoming point exactly once (one hash lookup +
// an online-mean update) and never re-averages an already-merged centroid
// as if it were a single fresh point, so voxel centroids don't drift as
// more data accumulates. Memory scales with the number of unique voxels
// visited, not the number of points seen or frames processed.
class IncrementalVoxelMap
{
 public:
  explicit IncrementalVoxelMap(double leafSize) : leafSize_(leafSize) {}

  void addPoint(const PointType &p)
  {
    VoxelKey key{
        static_cast<int64_t>(std::floor(p.x / leafSize_)),
        static_cast<int64_t>(std::floor(p.y / leafSize_)),
        static_cast<int64_t>(std::floor(p.z / leafSize_)),
    };
    VoxelData &d = voxels_[key];
    d.count++;
    // Sums kept in double regardless of PointType's float storage: a voxel
    // that's revisited thousands of times over a long run would otherwise
    // accumulate meaningful float rounding error.
    d.sumX += p.x;
    d.sumY += p.y;
    d.sumZ += p.z;
    d.sumIntensity += p.intensity;
  }

  void addCloud(const PointCloudXYZI &cloud)
  {
    for (const auto &p : cloud.points) addPoint(p);
  }

  size_t voxelCount() const { return voxels_.size(); }

  // Materializes the current per-voxel centroids. O(voxelCount()); meant to
  // be called once at the end (or occasionally for a live view), not per
  // point -- addPoint/addCloud are the cheap, per-point-safe operations.
  PointCloudXYZI::Ptr toCloud() const
  {
    PointCloudXYZI::Ptr out(new PointCloudXYZI());
    out->reserve(voxels_.size());
    for (const auto &[key, d] : voxels_)
    {
      PointType p;
      p.x = static_cast<float>(d.sumX / d.count);
      p.y = static_cast<float>(d.sumY / d.count);
      p.z = static_cast<float>(d.sumZ / d.count);
      p.intensity = static_cast<float>(d.sumIntensity / d.count);
      out->push_back(p);
    }
    return out;
  }

 private:
  struct VoxelKey
  {
    int64_t x, y, z;
    bool operator==(const VoxelKey &o) const { return x == o.x && y == o.y && z == o.z; }
  };

  struct VoxelKeyHash
  {
    size_t operator()(const VoxelKey &k) const
    {
      // boost::hash_combine-style mixing.
      size_t seed = std::hash<int64_t>{}(k.x);
      seed ^= std::hash<int64_t>{}(k.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int64_t>{}(k.z) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  struct VoxelData
  {
    uint64_t count = 0;
    double sumX = 0, sumY = 0, sumZ = 0, sumIntensity = 0;
  };

  double leafSize_;
  std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash> voxels_;
};

}  // namespace fastlio_standalone

#endif
