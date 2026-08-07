#ifndef FASTLIO_STANDALONE_ROS1_DESERIALIZE_H
#define FASTLIO_STANDALONE_ROS1_DESERIALIZE_H

// Minimal decoder for the raw bytes mcap hands back for "ros1" message
// encoding / "ros1msg" schema encoding channels. ROS1's wire format is a
// simple, undocumented-but-stable sequential little-endian struct packing:
// fixed fields inline, strings as (uint32 length, bytes), variable-length
// arrays as (uint32 count, elements). This intentionally only decodes the
// two message types fastlio needs (sensor_msgs/PointCloud2, sensor_msgs/Imu)
// -- there is no general-purpose .msg schema parser here.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

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

struct RosTime
{
  uint32_t sec = 0;
  uint32_t nsec = 0;
  double toSec() const { return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9; }
};

struct Header
{
  uint32_t seq;
  RosTime stamp;
  std::string frame_id;
};

inline RosTime readTime(Reader &r)
{
  RosTime t;
  t.sec = r.readU32();
  t.nsec = r.readU32();
  return t;
}

inline Header readHeader(Reader &r)
{
  Header h;
  h.seq = r.readU32();
  h.stamp = readTime(r);
  h.frame_id = r.readString();
  return h;
}

// sensor_msgs/PointField datatype constants.
enum PointFieldType : uint8_t
{
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
};

struct PointFieldDesc
{
  std::string name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;
};

struct PointCloud2Raw
{
  Header header;
  uint32_t height = 0, width = 0;
  std::vector<PointFieldDesc> fields;
  bool is_bigendian = false;
  uint32_t point_step = 0, row_step = 0;
  std::vector<std::byte> data;
  bool is_dense = true;
};

inline PointCloud2Raw readPointCloud2(Reader &r)
{
  PointCloud2Raw pc;
  pc.header = readHeader(r);
  pc.height = r.readU32();
  pc.width = r.readU32();

  uint32_t num_fields = r.readU32();
  pc.fields.reserve(num_fields);
  for (uint32_t i = 0; i < num_fields; i++)
  {
    PointFieldDesc f;
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

// Reads a single PointField's value out of one point's raw bytes, widened to
// double regardless of its wire datatype (every case fits losslessly: the
// largest integer type here is 32-bit, well within double's exact range).
inline double readFieldAsDouble(const std::byte *pointBase, const PointFieldDesc &f)
{
  const std::byte *p = pointBase + f.offset;
  switch (f.datatype)
  {
    case INT8:
    {
      int8_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case UINT8:
    {
      uint8_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case INT16:
    {
      int16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case UINT16:
    {
      uint16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case INT32:
    {
      int32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case UINT32:
    {
      uint32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case FLOAT32:
    {
      float v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case FLOAT64:
    {
      double v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    default:
      throw std::runtime_error("ros1msg: unsupported PointField datatype " + std::to_string(f.datatype));
  }
}

struct ImuRaw
{
  Header header;
  double angular_velocity[3];
  double linear_acceleration[3];
};

inline ImuRaw readImu(Reader &r)
{
  ImuRaw imu;
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
