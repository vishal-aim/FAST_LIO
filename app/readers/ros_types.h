#ifndef FASTLIO_APP_READERS_ROS_TYPES_H
#define FASTLIO_APP_READERS_ROS_TYPES_H

// Decoded message shapes shared between the ROS1 (raw packed) and ROS2 (CDR)
// wire-format decoders (ros1_deserialize.h / ros2_cdr_deserialize.h). Both
// sensor_msgs/PointCloud2 and sensor_msgs/Imu have the same fields, in the
// same order, in ROS1 and ROS2 -- only the byte-level encoding differs -- so
// a single decoded struct per message type keeps everything downstream
// (sensor layout detection, Preprocess dispatch) encoding-agnostic.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace rosmsg
{

// ROS1's Header.stamp is {uint32 secs, uint32 nsecs}; ROS2's is
// {int32 sec, uint32 nanosec} (builtin_interfaces/Time). int64_t holds
// either losslessly.
struct RosTime
{
  int64_t sec = 0;
  uint32_t nanosec = 0;
  double toSec() const { return static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9; }
};

struct Header
{
  uint32_t seq = 0;  // ROS1 only; ROS2's std_msgs/Header has no seq field, left 0
  RosTime stamp;
  std::string frame_id;
};

// sensor_msgs/PointField datatype constants (identical values in ROS1/ROS2).
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

struct ImuRaw
{
  Header header;
  double angular_velocity[3];
  double linear_acceleration[3];
};

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
      throw std::runtime_error("rosmsg: unsupported PointField datatype " + std::to_string(f.datatype));
  }
}

}  // namespace rosmsg

#endif
