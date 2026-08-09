#ifndef FASTLIO_APP_READERS_ROS1_DESERIALIZE_H
#define FASTLIO_APP_READERS_ROS1_DESERIALIZE_H

// Decoder for "ros1"-encoded message bytes (schema encoding "ros1msg").
// ROS1's wire format is a simple, undocumented-but-stable sequential
// little-endian struct packing: fixed fields inline with no alignment
// padding, strings as (uint32 length, bytes), variable-length arrays as
// (uint32 count, elements). This intentionally only decodes the two message
// types fastlio needs (sensor_msgs/PointCloud2, sensor_msgs/Imu) -- there is
// no general-purpose .msg schema parser here. See ros2_cdr_deserialize.h for
// the ROS2 equivalent -- both produce the shared structs in ros_types.h.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "ros_types.h"

namespace ros1msg
{

class Reader
{
 public:
  Reader(const std::byte *data, size_t size) : data_(data), size_(size) {}

  uint8_t readU8() { return readPod<uint8_t>(); }
  bool readBool() { return readU8() != 0; }
  uint16_t readU16() { return readPod<uint16_t>(); }
  uint32_t readU32() { return readPod<uint32_t>(); }
  uint64_t readU64() { return readPod<uint64_t>(); }
  float readF32() { return readPod<float>(); }
  double readF64() { return readPod<double>(); }

  std::string readString()
  {
    uint32_t len = readU32();
    require(len);
    std::string s(reinterpret_cast<const char *>(data_ + pos_), len);
    pos_ += len;
    return s;
  }

  void skip(size_t n)
  {
    require(n);
    pos_ += n;
  }

  const std::byte *cursor() const { return data_ + pos_; }
  size_t remaining() const { return size_ - pos_; }

 private:
  template <typename T>
  T readPod()
  {
    require(sizeof(T));
    T value;
    std::memcpy(&value, data_ + pos_, sizeof(T));
    pos_ += sizeof(T);
    return value;
  }

  void require(size_t n) const
  {
    if (pos_ + n > size_)
    {
      throw std::runtime_error("ros1msg::Reader: read past end of message buffer");
    }
  }

  const std::byte *data_;
  size_t size_;
  size_t pos_ = 0;
};

inline rosmsg::Header readHeader(Reader &r)
{
  rosmsg::Header h;
  h.seq = r.readU32();
  h.stamp.sec = r.readU32();
  h.stamp.nanosec = r.readU32();
  h.frame_id = r.readString();
  return h;
}

inline rosmsg::PointCloud2Raw readPointCloud2(Reader &r)
{
  rosmsg::PointCloud2Raw pc;
  pc.header = readHeader(r);
  pc.height = r.readU32();
  pc.width = r.readU32();

  uint32_t num_fields = r.readU32();
  pc.fields.reserve(num_fields);
  for (uint32_t i = 0; i < num_fields; i++)
  {
    rosmsg::PointFieldDesc f;
    f.name = r.readString();
    f.offset = r.readU32();
    f.datatype = r.readU8();
    f.count = r.readU32();
    pc.fields.push_back(std::move(f));
  }

  pc.is_bigendian = r.readBool();
  pc.point_step = r.readU32();
  pc.row_step = r.readU32();

  uint32_t data_len = r.readU32();
  pc.data.resize(data_len);
  std::memcpy(pc.data.data(), r.cursor(), data_len);
  r.skip(data_len);

  pc.is_dense = r.readBool();
  return pc;
}

inline rosmsg::ImuRaw readImu(Reader &r)
{
  rosmsg::ImuRaw imu;
  imu.header = readHeader(r);

  r.skip(4 * sizeof(double));  // orientation (x,y,z,w)
  r.skip(9 * sizeof(double));  // orientation_covariance

  for (double &v : imu.angular_velocity) v = r.readF64();
  r.skip(9 * sizeof(double));  // angular_velocity_covariance

  for (double &v : imu.linear_acceleration) v = r.readF64();
  r.skip(9 * sizeof(double));  // linear_acceleration_covariance

  return imu;
}

}  // namespace ros1msg

#endif
