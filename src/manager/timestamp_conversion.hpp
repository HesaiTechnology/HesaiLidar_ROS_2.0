#ifndef HESAI_ROS_TIMESTAMP_CONVERSION_HPP
#define HESAI_ROS_TIMESTAMP_CONVERSION_HPP

#include <cmath>
#include <cstdint>
#include <limits>

namespace hesai_ros {

// Stamp follows builtin_interfaces/Time: signed seconds and normalized nanoseconds.
template <typename Stamp>
bool AssignTimestamp(double timestamp, Stamp& stamp) {
  stamp.sec = 0;
  stamp.nanosec = 0;
  if (!std::isfinite(timestamp)) return false;
  const double floor_seconds = std::floor(timestamp);
  if (floor_seconds < std::numeric_limits<int32_t>::min() ||
      floor_seconds > std::numeric_limits<int32_t>::max()) return false;
  int64_t seconds = static_cast<int64_t>(floor_seconds);
  int64_t nanoseconds = std::llround((timestamp - floor_seconds) * 1e9);
  if (nanoseconds == 1000000000LL) {
    ++seconds;
    nanoseconds = 0;
  }
  if (seconds > std::numeric_limits<int32_t>::max()) return false;
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds);
  return true;
}

}  // namespace hesai_ros
#endif
