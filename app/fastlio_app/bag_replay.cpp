#include "bag_replay.h"

#include <iostream>
#include <optional>
#include <stdexcept>

#include "readers/point_field_reader.h"

namespace fastlio_app
{

namespace
{

size_t pointCount(const aimcap::DecodedPointCloud &pc)
{
  if (pc.point_step == 0) return 0;
  return pc.data.size() / pc.point_step;
}

void decodeVelodyne(const aimcap::DecodedPointCloud &pc, pcl::PointCloud<velodyne_ros::Point> &out)
{
  const auto *fx = findField(pc.fields, "x");
  const auto *fy = findField(pc.fields, "y");
  const auto *fz = findField(pc.fields, "z");
  const auto *fi = findField(pc.fields, "intensity");
  const auto *ft = findField(pc.fields, "time");
  const auto *fr = findField(pc.fields, "ring");
  if (!fx || !fy || !fz || !ft || !fr)
  {
    throw std::runtime_error("PointCloud missing a required velodyne field (x/y/z/time/ring)");
  }

  size_t n = pointCount(pc);
  out.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const uint8_t *base = pc.data.data() + i * pc.point_step;
    velodyne_ros::Point &p = out.points[i];
    p.x = static_cast<float>(readFieldAsDouble(base, *fx));
    p.y = static_cast<float>(readFieldAsDouble(base, *fy));
    p.z = static_cast<float>(readFieldAsDouble(base, *fz));
    p.intensity = fi ? static_cast<float>(readFieldAsDouble(base, *fi)) : 0.f;
    p.time = static_cast<float>(readFieldAsDouble(base, *ft));
    p.ring = static_cast<uint16_t>(readFieldAsDouble(base, *fr));
  }
}

// reflectivity/ring/ambient/range are all optional (default 0 if absent): a
// raw foxglove.PointCloud channel (aimcap's default decode of an
// uncompressed AIM capture) only carries x/y/z/reflectivity plus the
// column-timestamp "t" field aimcap adds -- no ring/ambient/range at all.
// Preprocess::oust64_handler's actual per-point math (see
// libs/fastlio/src/preprocess.cpp) only reads x/y/z/intensity/t in the
// feature_extract_enable=false path this app uses -- ring/ambient/range
// exist on ouster_ros::Point only for driver-struct compatibility, so
// defaulting them to 0 here doesn't affect FAST-LIO's own math, but "t" is
// exactly the per-point deskewing timestamp and must always be real.
void decodeOuster(const aimcap::DecodedPointCloud &pc, pcl::PointCloud<ouster_ros::Point> &out)
{
  const auto *fx = findField(pc.fields, "x");
  const auto *fy = findField(pc.fields, "y");
  const auto *fz = findField(pc.fields, "z");
  const auto *fi = findField(pc.fields, "intensity");
  const auto *frefl = findField(pc.fields, "reflectivity");  // intensity fallback if "intensity" is absent
  const auto *ft = findField(pc.fields, "t");
  const auto *fring = findField(pc.fields, "ring");
  const auto *famb = findField(pc.fields, "ambient");
  const auto *frange = findField(pc.fields, "range");
  if (!fx || !fy || !fz || !ft)
  {
    throw std::runtime_error("PointCloud missing a required ouster field (x/y/z/t)");
  }

  size_t n = pointCount(pc);
  out.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const uint8_t *base = pc.data.data() + i * pc.point_step;
    ouster_ros::Point &p = out.points[i];
    p.x = static_cast<float>(readFieldAsDouble(base, *fx));
    p.y = static_cast<float>(readFieldAsDouble(base, *fy));
    p.z = static_cast<float>(readFieldAsDouble(base, *fz));
    p.intensity = fi   ? static_cast<float>(readFieldAsDouble(base, *fi))
                  : frefl ? static_cast<float>(readFieldAsDouble(base, *frefl))
                          : 0.f;
    p.t = static_cast<uint32_t>(readFieldAsDouble(base, *ft));
    p.reflectivity = frefl ? static_cast<uint16_t>(readFieldAsDouble(base, *frefl)) : 0;
    p.ring = fring ? static_cast<uint8_t>(readFieldAsDouble(base, *fring)) : 0;
    p.ambient = famb ? static_cast<uint16_t>(readFieldAsDouble(base, *famb)) : 0;
    p.range = frange ? static_cast<uint32_t>(readFieldAsDouble(base, *frange)) : 0;
  }
}

void decodeGenericXYZI(const aimcap::DecodedPointCloud &pc, pcl::PointCloud<pcl::PointXYZI> &out)
{
  const auto *fx = findField(pc.fields, "x");
  const auto *fy = findField(pc.fields, "y");
  const auto *fz = findField(pc.fields, "z");
  const auto *fi = findField(pc.fields, "intensity");
  if (!fx || !fy || !fz)
  {
    throw std::runtime_error("PointCloud missing a required field (x/y/z)");
  }

  size_t n = pointCount(pc);
  out.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const uint8_t *base = pc.data.data() + i * pc.point_step;
    pcl::PointXYZI &p = out.points[i];
    p.x = static_cast<float>(readFieldAsDouble(base, *fx));
    p.y = static_cast<float>(readFieldAsDouble(base, *fy));
    p.z = static_cast<float>(readFieldAsDouble(base, *fz));
    p.intensity = fi ? static_cast<float>(readFieldAsDouble(base, *fi)) : 0.f;
  }
}

void decodeAndPreprocess(const aimcap::DecodedPointCloud &pc, Preprocess &preprocess, PointCloudXYZI::Ptr &out)
{
  if (hasField(pc.fields, "ring") && hasField(pc.fields, "time") && !hasField(pc.fields, "t"))
  {
    pcl::PointCloud<velodyne_ros::Point> cloud;
    decodeVelodyne(pc, cloud);
    preprocess.process(cloud, out);
  }
  else if (hasField(pc.fields, "t"))
  {
    // "t" (per-point deskewing timestamp) is the only field
    // Preprocess::oust64_handler's non-feature-extraction path actually
    // reads besides x/y/z/intensity -- ring/reflectivity/ambient/range may
    // or may not be present (a raw, uncompressed AIM foxglove.PointCloud
    // only ever carries x/y/z/reflectivity/t), decodeOuster() defaults
    // those to 0 when absent.
    pcl::PointCloud<ouster_ros::Point> cloud;
    decodeOuster(pc, cloud);
    preprocess.process(cloud, out);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    decodeGenericXYZI(pc, cloud);
    preprocess.process(cloud, out);
  }
}

double pointCloudStampSec(const aimcap::DecodedPointCloud &pc)
{
  return static_cast<double>(pc.stamp_sec) + static_cast<double>(pc.stamp_nsec) * 1e-9;
}

double imuStampSec(const aimcap::DecodedImu &imu)
{
  return static_cast<double>(imu.time_ns) * 1e-9;
}

}  // namespace

void replayBag(aimcap::Reader &reader,
                const std::string &lidTopic,
                const std::string &imuTopic,
                double timeOffsetLidarToImu,
                Preprocess &preprocess,
                fastlio::PacketSync &sync,
                const std::function<void(const MeasureGroup &)> &onMeasurement,
                const BagTimeRange &range)
{
  size_t lidarCount = 0, imuCount = 0;
  // Must persist across calls: nextMeasurement() can stage meas.lidar and
  // return false while it waits for enough IMU coverage, then finish the
  // same measurement on a later call once more IMU data has arrived. A
  // fresh MeasureGroup per drain would silently discard that staged lidar
  // scan (mirrors the original ROS node's single persistent `Measures`).
  MeasureGroup meas;
  auto drainReady = [&]() {
    while (sync.nextMeasurement(meas))
    {
      onMeasurement(meas);
    }
  };

  // aimcap::Reader has no "restrict to a time range" replay mode -- it
  // always decodes every subscribed topic across every input file. The
  // bag's own first message (lidar or IMU, whichever comes first) becomes
  // the reference point for --start-time/--duration; messages outside the
  // resulting window are decoded (unavoidable -- aimcap:: already decoded
  // them by the time this callback runs) but not fed to sync/preprocess.
  //
  // Only actually compute/apply this when a restriction was requested:
  // lidar and IMU each carry their own sensor clock, offset from each other
  // by a small amount, so anchoring bagStartSec on whichever topic's first
  // message happens to be processed first (IMU, at 100Hz vs lidar's ~19Hz)
  // can make the *other* topic's very first message appear to be at a
  // negative relative time and get silently dropped -- not a real filter,
  // just clock skew between streams. Skipping this entirely when no range
  // was asked for avoids that off-by-one-scan trap.
  const bool restrictRange = range.startTimeSec.has_value() || range.endTimeSec.has_value();
  std::optional<double> bagStartSec;
  auto inRange = [&](double stampSec) {
    if (!restrictRange) return true;
    if (!bagStartSec) bagStartSec = stampSec;
    double rel = stampSec - *bagStartSec;
    if (rel < range.startTimeSec.value_or(0.0)) return false;
    if (range.endTimeSec && rel >= *range.endTimeSec) return false;
    return true;
  };

  reader.OnPointCloud(lidTopic, [&](const std::string & /*topic*/, const aimcap::DecodedPointCloud &pc) {
    double stampSec = pointCloudStampSec(pc);
    if (!inRange(stampSec)) return;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    decodeAndPreprocess(pc, preprocess, ptr);
    if (!sync.pushLidar(ptr, stampSec))
    {
      std::cerr << "[bag] lidar loop back, clear buffer" << std::endl;
    }
    lidarCount++;
    drainReady();
  });

  reader.OnImu(imuTopic, [&](const std::string & /*topic*/, const aimcap::DecodedImu &imu) {
    double stampSec = imuStampSec(imu);
    if (!inRange(stampSec)) return;

    fastlio::ImuSample sample;
    sample.timestamp = stampSec - timeOffsetLidarToImu;
    sample.acc = V3D(imu.linear_acceleration_mps2[0], imu.linear_acceleration_mps2[1], imu.linear_acceleration_mps2[2]);
    sample.gyro = V3D(imu.angular_velocity_radps[0], imu.angular_velocity_radps[1], imu.angular_velocity_radps[2]);
    if (!sync.pushImu(sample))
    {
      std::cerr << "[bag] imu loop back, clear buffer" << std::endl;
    }
    imuCount++;
    drainReady();
  });

  reader.Run();

  std::cout << "[bag] replayed " << lidarCount << " lidar scans, " << imuCount << " imu samples" << std::endl;
  if (lidarCount == 0)
  {
    std::cerr << "[bag] warning: no messages found on lidar topic '" << lidTopic << "'" << std::endl;
  }
  if (imuCount == 0)
  {
    std::cerr << "[bag] warning: no messages found on imu topic '" << imuTopic << "'" << std::endl;
  }
}

}  // namespace fastlio_app
