#include "../src/manager/timestamp_conversion.hpp"
#include <cassert>
#include <iostream>

struct Stamp { int32_t sec; uint32_t nanosec; };

int main() {
  Stamp stamp;
  int cases = 0;
  const auto check = [&](double input, int32_t sec, uint32_t nanosec) {
    assert(hesai_ros::AssignTimestamp(input, stamp));
    assert(stamp.sec == sec && stamp.nanosec == nanosec);
    assert(stamp.nanosec < 1000000000U);
    ++cases;
  };
  check(0, 0, 0);
  check(1.7, 1, 700000000);
  check(1.9999999996, 2, 0);
  check(-1.7, -2, 300000000);
  check(-0.0000000004, 0, 0);
  check(1700000000.0, 1700000000, 0);
  check(2147483647.5, 2147483647, 500000000);
  check(-2147483648.0, -2147483647-1, 0);
  for (double input : {2147483648.0, -2147483649.0,
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity()}) {
    stamp.sec = 7; stamp.nanosec = 9;
    assert(!hesai_ros::AssignTimestamp(input, stamp));
    assert(stamp.sec == 0 && stamp.nanosec == 0);
    ++cases;
  }
  // The former arithmetic produces a nanosecond field outside ROS Time's range.
  const double timestamp = 1.9999999996;
  const uint32_t old_sec = static_cast<uint32_t>(std::floor(timestamp));
  const uint32_t old_nsec = static_cast<uint32_t>(std::round((timestamp-old_sec)*1e9));
  assert(old_nsec == 1000000000U);
  ++cases;
  std::cout << cases << " timestamp cases passed; legacy overflow reproduced\n";
}
