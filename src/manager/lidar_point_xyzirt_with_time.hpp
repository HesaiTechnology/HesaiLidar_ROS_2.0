#pragma once

#include <cstdint>

namespace hesai_ros_driver
{

// Keep the legacy packet timestamp and carry precise point time when a parser
// provides it, so ROS 1 output can preserve existing behavior for other models.
struct LidarPointXYZIRTWithTime
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint16_t ring;
  double timestamp;
  std::uint64_t timeSecond;
  std::uint32_t timeNanosecond;
};

inline double PointTimestampSeconds(const LidarPointXYZIRTWithTime& point, bool use_point_time)
{
  if (!use_point_time) {
    return point.timestamp;
  }
  return static_cast<double>(point.timeSecond) +
      static_cast<double>(point.timeNanosecond) / 1e9;
}

}  // namespace hesai_ros_driver
