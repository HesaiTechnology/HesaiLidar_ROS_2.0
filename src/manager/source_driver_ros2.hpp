/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: source_driver_ros2.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sstream>
#include <hesai_ros_driver/msg/udp_frame.hpp>
#include <hesai_ros_driver/msg/udp_packet.hpp>
#include <hesai_ros_driver/msg/ptp.hpp>
#include <hesai_ros_driver/msg/firetime.hpp>
#include <hesai_ros_driver/msg/loss_packet.hpp>

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <boost/thread.hpp>
#include "source_drive_common.hpp"

class SourceDriver
{
public:
  typedef std::shared_ptr<SourceDriver> Ptr;
  // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
  virtual void Init(const YAML::Node& config);
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop();
  virtual ~SourceDriver();
  SourceDriver(SourceType src_type) {};
  void SpinRos2(){rclcpp::spin(this->node_ptr_);}
  std::shared_ptr<rclcpp::Node> node_ptr_;
  #ifdef __CUDACC__
    std::shared_ptr<HesaiLidarSdkGpu<LidarPointXYZIRT>> driver_ptr_;
  #else
    std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
  #endif
protected:
  // Save Correction file subscribed by "ros_recv_correction_topic"
  void RecieveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  // Save packets subscribed by 'ros_recv_packet_topic'
  void RecievePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original pcake through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t&  ros_msg, double timestamp);

  // Used to publish the Correction file through 'ros_send_correction_topic'
  void SendCorrection(const u8Array_t& msg);
  // Used to publish the Packet loss condition
  void SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Used to publish the Packet loss condition
  void SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Used to publish the firetime correction 
  void SendFiretime(const double *firetime_correction_);

  // Convert ptp lock offset, status into ROS message
  hesai_ros_driver::msg::Ptp ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status);
  // Convert packet loss condition into ROS message
  hesai_ros_driver::msg::LossPacket ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count);
  // Convert correction string into ROS messages
  std_msgs::msg::UInt8MultiArray ToRosMsg(const u8Array_t& correction_string);
  // Convert double[512] to float64[512]
  hesai_ros_driver::msg::Firetime ToRosMsg(const double *firetime_correction_);
  // Convert point clouds into ROS messages
  sensor_msgs::msg::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  // Convert packets into ROS messages
  hesai_ros_driver::msg::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg, double timestamp);
  std::string frame_id_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_sub_;
  rclcpp::Subscription<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_sub_;
  rclcpp::Publisher<hesai_ros_driver::msg::UdpFrame>::SharedPtr pkt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Firetime>::SharedPtr firetime_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr crt_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::LossPacket>::SharedPtr loss_pub_;
  rclcpp::Publisher<hesai_ros_driver::msg::Ptp>::SharedPtr ptp_pub_;

  //spin thread while recieve data from ROS topic
  boost::thread* subscription_spin_thread_;
};
inline void SourceDriver::Init(const YAML::Node& config)
{
  DriverParam driver_param;
  DriveYamlParam yaml_param;
  yaml_param.GetDriveYamlParam(config, driver_param);
  frame_id_ = driver_param.input_param.frame_id;

  node_ptr_.reset(new rclcpp::Node("hesai_ros_driver_node"));
  if (driver_param.input_param.send_point_cloud_ros) {
    pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(driver_param.input_param.ros_send_point_topic, 100);
  }


  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
    loss_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::LossPacket>(driver_param.input_param.ros_send_packet_loss_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
      ptp_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Ptp>(driver_param.input_param.ros_send_ptp_topic, 10);
    }

    if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
      crt_pub_ = node_ptr_->create_publisher<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_send_correction_topic, 10);
    }
  }
  if (! driver_param.input_param.firetimes_path.empty() ) {
    if (driver_param.input_param.ros_send_firetime_topic != NULL_TOPIC) {
      firetime_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::Firetime>(driver_param.input_param.ros_send_firetime_topic, 10);
    } 
  }

  if (driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET) {
    pkt_pub_ = node_ptr_->create_publisher<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_send_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    pkt_sub_ = node_ptr_->create_subscription<hesai_ros_driver::msg::UdpFrame>(driver_param.input_param.ros_recv_packet_topic, 10, 
    std::bind(&SourceDriver::RecievePacket, this, std::placeholders::_1));
  if (driver_param.input_param.ros_recv_correction_topic != NULL_TOPIC) {    
    crt_sub_ = node_ptr_->create_subscription<std_msgs::msg::UInt8MultiArray>(driver_param.input_param.ros_recv_correction_topic, 10, 
    std::bind(&SourceDriver::RecieveCorrection, this, std::placeholders::_1));
  }
    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos2,this));
  }
  #ifdef __CUDACC__
    driver_ptr_.reset(new HesaiLidarSdkGpu<LidarPointXYZIRT>());
    driver_param.decoder_param.enable_parser_thread = false;
  #else
    driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
    driver_param.decoder_param.enable_parser_thread = true;
  #endif
  driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPointCloud, this, std::placeholders::_1));
  if(driver_param.input_param.send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET){
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2)) ;
  }
  if (driver_param.input_param.ros_send_packet_loss_topic != NULL_TOPIC) {
  driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacketLoss, this, std::placeholders::_1, std::placeholders::_2));
}
  if (driver_param.input_param.source_type == DATA_FROM_LIDAR) {
if (driver_param.input_param.ros_send_correction_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendCorrection, this, std::placeholders::_1));
}
    if (driver_param.input_param.ros_send_ptp_topic != NULL_TOPIC) {
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPTP, this, std::placeholders::_1, std::placeholders::_2));
}
  } 
  if (!driver_ptr_->Init(driver_param))
  {
    std::cout << "Driver Initialize Error...." << std::endl;
    exit(-1);
  }
}

inline void SourceDriver::Start()
{
  driver_ptr_->Start();
}

inline SourceDriver::~SourceDriver()
{
  Stop();
}

inline void SourceDriver::Stop()
{
  driver_ptr_->Stop();
}

inline void SourceDriver::SendPacket(const UdpFrame_t& msg, double timestamp)
{
  pkt_pub_->publish(ToRosMsg(msg, timestamp));
}

inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  pub_->publish(ToRosMsg(msg, frame_id_));
}

inline void SourceDriver::SendCorrection(const u8Array_t& msg)
{
  crt_pub_->publish(ToRosMsg(msg));
}

inline void SourceDriver::SendPacketLoss(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  loss_pub_->publish(ToRosMsg(total_packet_count, total_packet_loss_count));
}

inline void SourceDriver::SendPTP(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  ptp_pub_->publish(ToRosMsg(ptp_lock_offset, ptp_status));
}

inline void SourceDriver::SendFiretime(const double *firetime_correction_)
{
  firetime_pub_->publish(ToRosMsg(firetime_correction_));
}

inline sensor_msgs::msg::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  sensor_msgs::msg::PointCloud2 ros_msg;

  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  ros_msg.width = frame.points_num; 
  ros_msg.height = 1; 

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = false;
  ros_msg.data.resize(frame.points_num * ros_msg.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
  for (size_t i = 0; i < frame.points_num; i++)
  {
    LidarPointXYZIRT point = frame.points[i];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timestamp;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;   
  }
  // printf("HesaiLidar Runing Status [standby mode:%u]  |  [speed:%u]\n", frame.work_mode, frame.spin_speed);
  printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;
  std::cout.flush();
  ros_msg.header.stamp.sec = (uint32_t)floor(frame.points[0].timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((frame.points[0].timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = frame_id_;
  return ros_msg;
}

inline hesai_ros_driver::msg::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg, double timestamp) {
  hesai_ros_driver::msg::UdpFrame rs_msg;
  for (size_t i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::msg::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  rs_msg.header.stamp.sec = (uint32_t)floor(timestamp);
  rs_msg.header.stamp.nanosec = (uint32_t)round((timestamp - rs_msg.header.stamp.sec) * 1e9);
  rs_msg.header.frame_id = frame_id_;
  return rs_msg;
}

inline std_msgs::msg::UInt8MultiArray SourceDriver::ToRosMsg(const u8Array_t& correction_string) {
  auto msg = std::make_shared<std_msgs::msg::UInt8MultiArray>();
  msg->data.resize(correction_string.size());
  std::copy(correction_string.begin(), correction_string.end(), msg->data.begin());
  return *msg;
}

inline hesai_ros_driver::msg::LossPacket SourceDriver::ToRosMsg(const uint32_t& total_packet_count, const uint32_t& total_packet_loss_count)
{
  hesai_ros_driver::msg::LossPacket msg;
  msg.total_packet_count = total_packet_count;
  msg.total_packet_loss_count = total_packet_loss_count;  
  return msg;
}

inline hesai_ros_driver::msg::Ptp SourceDriver::ToRosMsg(const uint8_t& ptp_lock_offset, const u8Array_t& ptp_status)
{
  hesai_ros_driver::msg::Ptp msg;
  msg.ptp_lock_offset = ptp_lock_offset;
  std::copy(ptp_status.begin(), ptp_status.begin() + std::min(16ul, ptp_status.size()), msg.ptp_status.begin());
  return msg;
}

inline hesai_ros_driver::msg::Firetime SourceDriver::ToRosMsg(const double *firetime_correction_)
{
  hesai_ros_driver::msg::Firetime msg;
  std::copy(firetime_correction_, firetime_correction_ + 512, msg.data.begin());
  return msg;
}
inline void SourceDriver::RecievePacket(const hesai_ros_driver::msg::UdpFrame::SharedPtr msg)
{
  for (size_t i = 0; i < msg->packets.size(); i++) {
    driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg->packets[i].data[0], msg->packets[i].size);
  }
}

inline void SourceDriver::RecieveCorrection(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  driver_ptr_->lidar_ptr_->correction_string_.resize(msg->data.size());
  std::copy(msg->data.begin(), msg->data.end(), driver_ptr_->lidar_ptr_->correction_string_.begin());
  while (1) {
    if (! driver_ptr_->lidar_ptr_->LoadCorrectionFromROSbag()) {
      break;
    }
  }
}
