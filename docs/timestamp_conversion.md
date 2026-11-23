# Normalize ROS 2 timestamp carry and validate the signed seconds range

The ROS 2 conversions calculate seconds with floor() and nanoseconds with round(). For 1.9999999996 seconds, this produces {sec:1,nanosec:1000000000}; builtin_interfaces/Time requires nanosec in [0,1000000000).

## Run
```bash
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/timestamp_conversion_test.cpp -o /tmp/hesai_timestamp_test
/tmp/hesai_timestamp_test
```

## Behavior
- Add a shared timestamp conversion helper that carries rounded nanoseconds into seconds.
- Apply it to PointCloud2, UdpPacket, UdpFrame and Imu conversions.
- Handle valid signed ROS Time values and reject nonfinite or out-of-range input before integer casts.
- Log invalid values and leave a zero stamp, matching the existing error-path convention of the frame conversions.

## Input and result contract
The boundary is reproduced with the original arithmetic and fixed in standalone C++ tests. The report concerns conversion behavior; validation covers the helper and four source call sites. A full ROS 2 build is the next integration check.
AssignTimestamp(timestamp,stamp) accepts a stamp with signed int32 sec and uint32 nanosec fields. It normalizes a rounded 1e9 nanosecond value by incrementing seconds, checks the int32 range and returns true. It resets the output to zero and returns false for nonfinite/out-of-range input. Negative fractional seconds use floor-based ROS Time normalization. It converts the supplied time without changing the timestamp source or per-point timestamps. The test binary requires only a C++11 compiler.

## Tests
14 standalone C++ timestamp cases. See the tests directory.

---

# 规范 ROS2 时间戳进位并检查有符号秒范围

ROS2 转换用 floor() 取秒、round() 取纳秒。当输入为 1.9999999996 秒时，得到 {sec:1,nanosec:1000000000}；builtin_interfaces/Time 要求 nanosec 位于 [0,1000000000)。

## 运行
```bash
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/timestamp_conversion_test.cpp -o /tmp/hesai_timestamp_test
/tmp/hesai_timestamp_test
```

## 功能
- 增加统一转换函数，将四舍五入溢出的纳秒进位到秒。
- 用于 PointCloud2、UdpPacket、UdpFrame 和 Imu 四处转换。
- 支持有效的有符号 ROS Time，并在整数转换前检查非有限值和越界值。
- 异常值记录日志并保留零时间戳，沿用帧转换现有错误路径的处理方式。

## 输入与结果
已用原计算表达式复现边界问题，并通过独立 C++ 测试验证修复。当前验证覆盖转换函数和四处调用点；完整 ROS2 构建作为下一步集成检查。
AssignTimestamp(timestamp,stamp) 的 stamp 含 int32 sec、uint32 nanosec。函数将舍入到 1e9 的纳秒进位，检查 int32 秒范围，成功返回 true。非有限值或越界输入将输出清零并返回 false；负小数按 ROS Time 的 floor 规则表示。只转换所给时间，不切换时间源或修改逐点时间。测试仅需 C++11 编译器。

## 测试
14 项独立 C++ 时间戳用例，详见 tests 目录。
