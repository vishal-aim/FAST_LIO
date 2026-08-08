#ifndef FASTLIO_STANDALONE_ROS2_CDR_DESERIALIZE_H
#define FASTLIO_STANDALONE_ROS2_CDR_DESERIALIZE_H

// Decoder for "cdr"-encoded message bytes (ROS2's wire format, used by both
// rosbag2 sqlite3 (.db3) and ROS2-originated mcap recordings). Unlike ROS1's
// naive packed struct format, CDR aligns each primitive to its own size --
// a uint32 at a 4-byte boundary relative to the payload start, a
// uint64/double at 8 bytes -- and every message is prefixed by a 4-byte
// encapsulation header. Only plain (non-"parameter list"/XTypes-extensible)
// little-endian CDR is supported, which is what a plain, non-extensible
// message type like sensor_msgs/PointCloud2 or Imu uses by default; this
// intentionally only decodes those two message types, not a general CDR/IDL
// parser. See ros1_deserialize.h for the ROS1 equivalent -- both produce the
// shared structs in ros_types.h.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "ros_types.h"

namespace ros2cdr
{

class Reader
{
 public:
  Reader(const std::byte *data, size_t size) : data_(data), size_(size)
  {
    require(4);
    // Encapsulation header: 2-byte representation identifier + 2-byte
    // options. 0x00,0x01 = CDR_LE, the default for plain (non-extensible)
    // ROS2 message types.
    if (!(data_[0] == std::byte{0x00} && data_[1] == std::byte{0x01}))
    {
      throw std::runtime_error("ros2cdr::Reader: unsupported CDR encapsulation kind (expected CDR_LE)");
    }
    pos_ = 4;
    payloadStart_ = 4;
  }

  uint8_t readU8() { return readPod<uint8_t>(); }
  bool readBool() { return readU8() != 0; }
  uint16_t readU16() { return readPod<uint16_t>(); }
  uint32_t readU32() { return readPod<uint32_t>(); }
  int32_t readI32() { return readPod<int32_t>(); }
  uint64_t readU64() { return readPod<uint64_t>(); }
  float readF32() { return readPod<float>(); }
  double readF64() { return readPod<double>(); }

  // CDR strings: align(4), uint32 length INCLUDING the trailing null, then
  // `length` bytes (the last of which is the null terminator).
  std::string readString()
  {
    uint32_t len = readU32();
    if (len == 0) return std::string();
    require(len);
    std::string s(reinterpret_cast<const char *>(data_ + pos_), len - 1);  // drop the null terminator
    pos_ += len;
    return s;
  }

  // Fixed-size arrays (e.g. float64[9]) have no length prefix in CDR --
  // caller just reads N elements. This is only a documentation marker.
  void skip(size_t n)
  {
    require(n);
    pos_ += n;
  }

  // Bulk copy for a uint8[]/byte sequence: no per-element alignment (element
  // size is 1), so this is just a raw memcpy + advance -- the caller has
  // already consumed the sequence's own uint32 length prefix via readU32().
  void readBytes(void *dst, size_t n)
  {
    require(n);
    std::memcpy(dst, data_ + pos_, n);
    pos_ += n;
  }

  size_t remaining() const { return size_ - pos_; }

 private:
  void align(size_t alignment)
  {
    size_t rel = pos_ - payloadStart_;
    size_t rem = rel % alignment;
    if (rem != 0)
    {
      size_t pad = alignment - rem;
      require(pad);
      pos_ += pad;
    }
  }

  template <typename T>
  T readPod()
  {
    align(sizeof(T));
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
      throw std::runtime_error("ros2cdr::Reader: read past end of message buffer");
    }
  }

  const std::byte *data_;
  size_t size_;
  size_t pos_ = 0;
  size_t payloadStart_ = 0;
};

// std_msgs/msg/Header: {builtin_interfaces/Time stamp; string frame_id;}
// -- note ROS2 dropped ROS1's uint32 seq field entirely.
inline rosmsg::Header readHeader(Reader &r)
{
  rosmsg::Header h;
  h.stamp.sec = r.readI32();
  h.stamp.nanosec = r.readU32();
  h.frame_id = r.readString();
  return h;
}

// sensor_msgs/msg/PointField: {string name; uint32 offset; uint8 datatype; uint32 count;}
inline rosmsg::PointFieldDesc readPointField(Reader &r)
{
  rosmsg::PointFieldDesc f;
  f.name = r.readString();
  f.offset = r.readU32();
  f.datatype = r.readU8();
  f.count = r.readU32();
  return f;
}

inline rosmsg::PointCloud2Raw readPointCloud2(Reader &r)
{
  rosmsg::PointCloud2Raw pc;
  pc.header = readHeader(r);
  pc.height = r.readU32();
  pc.width = r.readU32();

  // PointField[] fields -- variable-length sequence: align(4) + uint32 count + elements.
  uint32_t num_fields = r.readU32();
  pc.fields.reserve(num_fields);
  for (uint32_t i = 0; i < num_fields; i++)
  {
    pc.fields.push_back(readPointField(r));
  }

  pc.is_bigendian = r.readBool();
  pc.point_step = r.readU32();
  pc.row_step = r.readU32();

  // uint8[] data -- variable-length sequence of a 1-byte element type, so no
  // per-element alignment/padding, just align(4) + uint32 count + raw bytes.
  uint32_t data_len = r.readU32();
  pc.data.resize(data_len);
  if (data_len > 0) r.readBytes(pc.data.data(), data_len);

  pc.is_dense = r.readBool();
  return pc;
}

inline rosmsg::ImuRaw readImu(Reader &r)
{
  rosmsg::ImuRaw imu;
  imu.header = readHeader(r);

  r.skip(4 * sizeof(double));  // orientation (x,y,z,w) -- fixed array, no length prefix
  r.skip(9 * sizeof(double));  // orientation_covariance

  for (double &v : imu.angular_velocity) v = r.readF64();
  r.skip(9 * sizeof(double));  // angular_velocity_covariance

  for (double &v : imu.linear_acceleration) v = r.readF64();
  r.skip(9 * sizeof(double));  // linear_acceleration_covariance

  return imu;
}

}  // namespace ros2cdr

#endif
