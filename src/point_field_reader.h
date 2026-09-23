#ifndef FASTLIO_POINT_FIELD_READER_H
#define FASTLIO_POINT_FIELD_READER_H

#include <aimcap/types.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace fastlio
{

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
      return 0.0;
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

} // namespace fastlio

#endif
