#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "udp1_4_parser.h"
#ifdef ROS2_FOUND
#include "source_driver_ros2.hpp"
#endif

using namespace hesai::lidar;

namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

// Two blocks cover every possible byte, with the JT128 flags/dual-return mode
// reported in PR #120. No network connection, capture, or calibration is needed.
UdpPacket MakePacket(uint8_t flags) {
  UdpPacket packet{};
  packet.buffer[0] = 0xee;
  packet.buffer[1] = 0xff;
  packet.buffer[2] = 1;
  packet.buffer[3] = 4;
  auto* header = reinterpret_cast<HS_LIDAR_HEADER_ME_V4*>(
      packet.buffer + sizeof(HS_LIDAR_PRE_HEADER));
  header->m_u8LaserNum = 128;
  header->m_u8BlockNum = 2;
  header->m_u8EchoCount = 1;
  header->m_u8EchoNum = 2;
  header->m_u8DistUnit = 4;
  header->m_u8Status = flags;
  packet.packet_len = header->packetSize();
  packet.recv_timestamp = 42000000;
  auto* body = reinterpret_cast<uint8_t*>(header + 1);
  for (int block = 0; block < 2; ++block) {
    auto* azimuth = body + block * (2 + 128 * header->unitSize());
    for (int channel = 0; channel < 128; ++channel) {
      auto* unit = azimuth + 2 + channel * header->unitSize();
      unit[0] = 0xe8;  // 1000 distance units, little endian
      unit[1] = 0x03;
      unit[2] = 37;  // reflectivity
      int optional = 3;
      if (hasConfidence(flags)) unit[optional++] = 17;
      if (hasWeightFactor(flags)) unit[optional] = block * 128 + channel;
    }
  }
  auto* tail = reinterpret_cast<HS_LIDAR_TAIL_ME_V4*>(
      body + (2 + 128 * header->unitSize()) * 2 + sizeof(HS_LIDAR_BODY_CRC_ME_V4));
  tail->m_u8ReturnMode = 0x39;
  return packet;
}

template <typename Point>
void Decode(LidarDecodedFrame<Point>& frame, uint8_t flags) {
  auto parser = std::make_unique<Udp1_4Parser<Point>>(STR_OTHER);
  frame.fParam.use_timestamp_type = 1;
  Require(parser->DecodePacket(frame, MakePacket(flags)) == 0, "packet decode failed");
  Require(parser->ComputeXYZI(frame, 0) == 0, "point decode failed");
  Require(frame.valid_points[0] == 256, "unexpected point count");
  Require(frame.return_mode == 0x39, "dual return mode lost");
  frame.points_num = frame.valid_points[0];
}

#ifdef ROS2_FOUND
class TestDriver : public SourceDriver {
 public:
  TestDriver() : SourceDriver(DATA_FROM_PCAP) {
    driver_ptr_ = std::make_shared<HesaiLidarSdk<LidarPointXYZJT128>>();
    driver_ptr_->lidar_ptr_ = std::make_shared<Lidar<LidarPointXYZJT128>>();
    Require(driver_ptr_->lidar_ptr_->SetLidarType("JT128") == 0, "JT128 selection failed");
  }
  using SourceDriver::ToRosMsg;
};

void CheckMessage(TestDriver& driver, const LidarDecodedFrame<LidarPointXYZJT128>& frame,
                  const LidarPointXYZJT128* points, bool jt128) {
  const auto msg = driver.ToRosMsg(frame, "test_lidar");
  Require(msg.width == 256 && msg.height == 1, "message shape changed");
  Require(msg.header.frame_id == "test_lidar", "frame id changed");
  Require(msg.fields.size() == (jt128 ? 7U : 6U), "unexpected field count");
  Require(msg.point_step == (jt128 ? 27U : 26U), "unexpected point size");
  Require(msg.row_step == msg.width * msg.point_step, "row size mismatch");
  Require(msg.data.size() == msg.row_step, "data size mismatch");
  if (jt128) {
    const auto& field = msg.fields.back();
    Require(field.name == "confidence" && field.offset == 26 && field.count == 1 &&
            field.datatype == sensor_msgs::msg::PointField::UINT8, "confidence field changed");
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> intensity(msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> ring(msg, "ring");
  sensor_msgs::PointCloud2ConstIterator<double> timestamp(msg, "timestamp");
  for (size_t i = 0; i < 256; ++i, ++x, ++y, ++z, ++intensity, ++ring, ++timestamp) {
    Require(*x == points[i].x && *y == points[i].y && *z == points[i].z, "coordinates changed");
    Require(*intensity == points[i].intensity && *ring == points[i].ring &&
            *timestamp == points[i].timestamp, "existing point fields changed");
    if (jt128) Require(msg.data[i * msg.point_step + 26] == i, "JT128 raw byte lost");
  }
}
#endif
}  // namespace

int main() {
  try {
    LidarDecodedFrame<LidarPointXYZICRT> old_frame(1, 256);
    Decode(old_frame, 0x23);
    for (size_t i = 0; i < 256; ++i)
      Require(old_frame.points[i].confidence == 0, "baseline no longer reproduces the bug");

    LidarDecodedFrame<LidarPointXYZJT128> frame(1, 256);
    Decode(frame, 0x23);
    for (size_t i = 0; i < 256; ++i) {
      const auto& point = frame.points[i];
      Require(point.confidence == 0 && point.weightFactor == i, "wrong SDK byte lane");
      Require(point.dirtyLevel == (i >> 6) && point.noiseLevel == (i & 0x3f),
              "contamination/noise bits changed");
    }

    // When both flags are present, the bit-4 confidence byte must not replace
    // the bit-5 JT128 byte, and its extra byte must not shift subsequent points.
    LidarDecodedFrame<LidarPointXYZJT128> both_flags(1, 256);
    Decode(both_flags, 0x33);
    for (size_t i = 0; i < 256; ++i)
      Require(both_flags.points[i].confidence == 17 && both_flags.points[i].weightFactor == i,
              "optional-byte offsets mixed up");
#ifdef ROS2_FOUND
    TestDriver driver;
    CheckMessage(driver, frame, frame.points, true);
    CheckMessage(driver, both_flags, both_flags.points, true);
    frame.fParam.frame_frequency = 5;
    frame.fParam.default_frame_frequency = 10;
    frame.multi_points_num = 256;
    frame.multi_frame_buffer = new uint8_t[256 * sizeof(LidarPointXYZJT128)];
    frame.multi_points = reinterpret_cast<LidarPointXYZJT128*>(frame.multi_frame_buffer);
    std::copy_n(frame.points, 256, frame.multi_points);
    CheckMessage(driver, frame, frame.multi_points, true);
    Require(driver.driver_ptr_->lidar_ptr_->SetLidarType("OT128") == 0, "OT128 selection failed");
    CheckMessage(driver, both_flags, both_flags.points, false);
    std::cout << "PASS: raw/multi-frame ROS serialization and non-JT128 layout\n";
#endif
    std::cout << "PASS: real SDK packet decoding, 256 byte values, flags 0x23/0x33, dual return\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
