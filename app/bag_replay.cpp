#include "bag_replay.h"

#include <iostream>
#include <stdexcept>

#include "readers/ros1_deserialize.h"
#include "readers/ros2_cdr_deserialize.h"

namespace fastlio_app
{

namespace
{

const rosmsg::PointFieldDesc *findField(const std::vector<rosmsg::PointFieldDesc> &fields, const std::string &name)
{
  for (const auto &f : fields)
  {
    if (f.name == name) return &f;
  }
  return nullptr;
}

bool hasField(const std::vector<rosmsg::PointFieldDesc> &fields, const std::string &name)
{
  return findField(fields, name) != nullptr;
}

size_t pointCount(const rosmsg::PointCloud2Raw &pc)
{
  if (pc.point_step == 0) return 0;
  return pc.data.size() / pc.point_step;
}

void decodeVelodyne(const rosmsg::PointCloud2Raw &pc, pcl::PointCloud<velodyne_ros::Point> &out)
{
  const auto *fx = findField(pc.fields, "x");
  const auto *fy = findField(pc.fields, "y");
  const auto *fz = findField(pc.fields, "z");
  const auto *fi = findField(pc.fields, "intensity");
  const auto *ft = findField(pc.fields, "time");
  const auto *fr = findField(pc.fields, "ring");
  if (!fx || !fy || !fz || !ft || !fr)
  {
    throw std::runtime_error("PointCloud2 missing a required velodyne field (x/y/z/time/ring)");
  }

  size_t n = pointCount(pc);
  out.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const std::byte *base = pc.data.data() + i * pc.point_step;
    velodyne_ros::Point &p = out.points[i];
    p.x = static_cast<float>(rosmsg::readFieldAsDouble(base, *fx));
    p.y = static_cast<float>(rosmsg::readFieldAsDouble(base, *fy));
    p.z = static_cast<float>(rosmsg::readFieldAsDouble(base, *fz));
    p.intensity = fi ? static_cast<float>(rosmsg::readFieldAsDouble(base, *fi)) : 0.f;
    p.time = static_cast<float>(rosmsg::readFieldAsDouble(base, *ft));
    p.ring = static_cast<uint16_t>(rosmsg::readFieldAsDouble(base, *fr));
  }
}

void decodeOuster(const rosmsg::PointCloud2Raw &pc, pcl::PointCloud<ouster_ros::Point> &out)
{
  const auto *fx = findField(pc.fields, "x");
  const auto *fy = findField(pc.fields, "y");
  const auto *fz = findField(pc.fields, "z");
  const auto *fi = findField(pc.fields, "intensity");
  const auto *ft = findField(pc.fields, "t");
  const auto *fring = findField(pc.fields, "ring");
  const auto *frefl = findField(pc.fields, "reflectivity");
  const auto *famb = findField(pc.fields, "ambient");
  const auto *frange = findField(pc.fields, "range");
  if (!fx || !fy || !fz || !ft || !fring || !frefl || !famb || !frange)
  {
    throw std::runtime_error("PointCloud2 missing a required ouster field (x/y/z/t/ring/reflectivity/ambient/range)");
  }

  size_t n = pointCount(pc);
  out.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const std::byte *base = pc.data.data() + i * pc.point_step;
    ouster_ros::Point &p = out.points[i];
    p.x = static_cast<float>(rosmsg::readFieldAsDouble(base, *fx));
    p.y = static_cast<float>(rosmsg::readFieldAsDouble(base, *fy));
    p.z = static_cast<float>(rosmsg::readFieldAsDouble(base, *fz));
    p.intensity = fi ? static_cast<float>(rosmsg::readFieldAsDouble(base, *fi)) : 0.f;
    p.t = static_cast<uint32_t>(rosmsg::readFieldAsDouble(base, *ft));
    p.reflectivity = static_cast<uint16_t>(rosmsg::readFieldAsDouble(base, *frefl));
    p.ring = static_cast<uint8_t>(rosmsg::readFieldAsDouble(base, *fring));
    p.ambient = static_cast<uint16_t>(rosmsg::readFieldAsDouble(base, *famb));
    p.range = static_cast<uint32_t>(rosmsg::readFieldAsDouble(base, *frange));
  }
}

void decodeGenericXYZI(const rosmsg::PointCloud2Raw &pc, pcl::PointCloud<pcl::PointXYZI> &out)
{
  const auto *fx = findField(pc.fields, "x");
  const auto *fy = findField(pc.fields, "y");
  const auto *fz = findField(pc.fields, "z");
  const auto *fi = findField(pc.fields, "intensity");
  if (!fx || !fy || !fz)
  {
    throw std::runtime_error("PointCloud2 missing a required field (x/y/z)");
  }

  size_t n = pointCount(pc);
  out.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    const std::byte *base = pc.data.data() + i * pc.point_step;
    pcl::PointXYZI &p = out.points[i];
    p.x = static_cast<float>(rosmsg::readFieldAsDouble(base, *fx));
    p.y = static_cast<float>(rosmsg::readFieldAsDouble(base, *fy));
    p.z = static_cast<float>(rosmsg::readFieldAsDouble(base, *fz));
    p.intensity = fi ? static_cast<float>(rosmsg::readFieldAsDouble(base, *fi)) : 0.f;
  }
}

void decodeAndPreprocess(const rosmsg::PointCloud2Raw &pc, Preprocess &preprocess, PointCloudXYZI::Ptr &out)
{
  if (hasField(pc.fields, "ring") && hasField(pc.fields, "time") && !hasField(pc.fields, "t"))
  {
    pcl::PointCloud<velodyne_ros::Point> cloud;
    decodeVelodyne(pc, cloud);
    preprocess.process(cloud, out);
  }
  else if (hasField(pc.fields, "t") && hasField(pc.fields, "reflectivity") && hasField(pc.fields, "ring") &&
           hasField(pc.fields, "ambient") && hasField(pc.fields, "range"))
  {
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

rosmsg::PointCloud2Raw decodePointCloud2(const RawMessage &msg)
{
  if (msg.encoding == "ros1")
  {
    ros1msg::Reader r(msg.data, msg.size);
    return ros1msg::readPointCloud2(r);
  }
  if (msg.encoding == "cdr")
  {
    ros2cdr::Reader r(msg.data, msg.size);
    return ros2cdr::readPointCloud2(r);
  }
  throw std::runtime_error("topic '" + msg.topic + "' uses encoding '" + msg.encoding +
                            "', only 'ros1' and 'cdr' are supported");
}

rosmsg::ImuRaw decodeImu(const RawMessage &msg)
{
  if (msg.encoding == "ros1")
  {
    ros1msg::Reader r(msg.data, msg.size);
    return ros1msg::readImu(r);
  }
  if (msg.encoding == "cdr")
  {
    ros2cdr::Reader r(msg.data, msg.size);
    return ros2cdr::readImu(r);
  }
  throw std::runtime_error("topic '" + msg.topic + "' uses encoding '" + msg.encoding +
                            "', only 'ros1' and 'cdr' are supported");
}

}  // namespace

void replayBag(BagSource &source,
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

  source.forEachMessage({lidTopic, imuTopic}, [&](const RawMessage &msg) {
    if (msg.topic == lidTopic)
    {
      rosmsg::PointCloud2Raw pc = decodePointCloud2(msg);
      PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
      decodeAndPreprocess(pc, preprocess, ptr);
      if (!sync.pushLidar(ptr, pc.header.stamp.toSec()))
      {
        std::cerr << "[bag] lidar loop back, clear buffer" << std::endl;
      }
      lidarCount++;
    }
    else if (msg.topic == imuTopic)
    {
      rosmsg::ImuRaw imu = decodeImu(msg);
      fastlio::ImuSample sample;
      sample.timestamp = imu.header.stamp.toSec() - timeOffsetLidarToImu;
      sample.acc = V3D(imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]);
      sample.gyro = V3D(imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
      if (!sync.pushImu(sample))
      {
        std::cerr << "[bag] imu loop back, clear buffer" << std::endl;
      }
      imuCount++;
    }

    drainReady();
  }, range);

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
