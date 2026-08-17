#ifndef FASTLIO_APP_READERS_POINT_FIELD_READER_H
#define FASTLIO_APP_READERS_POINT_FIELD_READER_H

// Generic byte-level reader for aimcap::PointField-described point data --
// shared between app/fastlio_app/bag_replay.cpp (sensor-specific decode:
// velodyne/ouster/generic XYZI, feeding fastlio::Preprocess) and
// app/patchwork_app/main.cpp (x/y/z/intensity only, feeding Patchwork++'s
// Eigen matrix) since both need the exact same "read one field's value out
// of one point's raw bytes" primitive. Each app's own higher-level decode
// stays self-contained rather than merging into a shared decode layer here.

#include <aimcap/types.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace fastlio_app
{

// Reads a single PointField's value out of one point's raw bytes, widened to
// double regardless of its wire datatype (every case fits losslessly: the
// largest integer type here is 32-bit, well within double's exact range).
inline double readFieldAsDouble(const uint8_t *pointBase, const aimcap::PointField &f)
{
  const uint8_t *p = pointBase + f.offset;
  switch (f.type)
  {
    case aimcap::PointFieldType::INT8:
    {
      int8_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::UINT8:
    {
      uint8_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::INT16:
    {
      int16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::UINT16:
    {
      uint16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::INT32:
    {
      int32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::UINT32:
    {
      uint32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::FLOAT32:
    {
      float v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case aimcap::PointFieldType::FLOAT64:
    {
      double v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    default:
      throw std::runtime_error("readFieldAsDouble: unsupported PointField datatype " +
                                std::to_string(static_cast<int>(f.type)));
  }
}

inline const aimcap::PointField *findField(const std::vector<aimcap::PointField> &fields, const std::string &name)
{
  for (const auto &f : fields)
  {
    if (f.name == name) return &f;
  }
  return nullptr;
}

inline bool hasField(const std::vector<aimcap::PointField> &fields, const std::string &name)
{
  return findField(fields, name) != nullptr;
}

}  // namespace fastlio_app

#endif
