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
 * File: source_driver_ros1.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Source Driver for ROS1
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "hesai_ros_driver/UdpFrame.h"
#include "hesai_ros_driver/UdpPacket.h"

#ifdef __CUDACC__
  #include "hesai_lidar_sdk_gpu.cuh"
#else
  #include "hesai_lidar_sdk.hpp"
#endif
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
  void SpinRos1() {
    ros::MultiThreadedSpinner spinner(2); 
    spinner.spin();
  }
protected:
  // Save packets subscribed by 'ros_recv_packet_topic'
  void RecievePacket(const hesai_ros_driver::UdpFrame& msg);
  // Used to publish point clouds through 'ros_send_point_cloud_topic'
  void SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg);
  // Used to publish the original pcake through 'ros_send_packet_topic'
  void SendPacket(const UdpFrame_t&  ros_msg);
  // Convert point clouds into ROS messages
  sensor_msgs::PointCloud2 ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id);
  // Convert packets into ROS messages
  hesai_ros_driver::UdpFrame ToRosMsg(const UdpFrame_t& ros_msg);

  #ifdef __CUDACC__
    std::shared_ptr<HesaiLidarSdkGpu<LidarPointXYZIRT>> driver_ptr_;
  #else
    std::shared_ptr<HesaiLidarSdk<LidarPointXYZIRT>> driver_ptr_;
  #endif
  // publish point
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  // publish packet 
  ros::Publisher pkt_pub_;
  // packet sub
  ros::Subscriber pkt_sub_;
  //spin thread while recieve data from ROS topic
  boost::thread* subscription_spin_thread_;
};


inline void SourceDriver::Init(const YAML::Node& config)
{
  YAML::Node driver_config = YamlSubNodeAbort(config, "driver");
  DriverParam driver_param;
  // input related
  YamlRead<uint16_t>(driver_config, "udp_port", driver_param.input_param.udp_port, 2368);
  YamlRead<uint16_t>(driver_config, "ptc_port", driver_param.input_param.ptc_port, 9347);
  YamlRead<std::string>(driver_config, "host_ip_address", driver_param.input_param.host_ip_address, "192.168.1.100");
  YamlRead<std::string>(driver_config, "group_address", driver_param.input_param.multicast_ip_address, "");
  YamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
  YamlRead<std::string>(driver_config, "firetimes_path",driver_param.input_param.firetimes_path, "");
  YamlRead<std::string>(driver_config, "correction_file_path", driver_param.input_param.correction_file_path, "");
  
  // decoder related
  YamlRead<bool>(driver_config, "pcap_play_synchronization", driver_param.decoder_param.pcap_play_synchronization, true);
  YamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
  YamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
  YamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
  YamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
  YamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
  YamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);
  YamlRead<std::string>(driver_config, "device_ip_address", driver_param.input_param.device_ip_address, "192.168.1.201");
  YamlRead<float>(driver_config, "frame_start_azimuth", driver_param.decoder_param.frame_start_azimuth, -1);
  
  int source_type = 0;
  YamlRead<int>(driver_config, "source_type", source_type, 0);
  driver_param.input_param.source_type = SourceType(source_type);
  bool send_packet_ros;
  YamlRead<bool>(config["ros"], "send_packet_ros", send_packet_ros, false);
  bool send_point_cloud_ros;
  YamlRead<bool>(config["ros"], "send_point_cloud_ros", send_point_cloud_ros, false);
  YamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "hesai_lidar");
  std::string ros_send_packet_topic;
  YamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_packet_topic, "hesai_packets");
  std::string ros_send_point_topic;
  YamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_point_topic, "hesai_points");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  if (send_point_cloud_ros) {
    std::string ros_send_point_topic;
    YamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_point_topic, "hesai_points");
    pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_point_topic, 10);
  }

  if (send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET) {
    std::string ros_send_packet_topic;
    YamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_packet_topic, "hesai_packets");
    pkt_pub_ = nh_->advertise<hesai_ros_driver::UdpFrame>(ros_send_packet_topic, 10);
  }

  if (driver_param.input_param.source_type == DATA_FROM_ROS_PACKET) {
    std::string ros_recv_packet_topic;
    YamlRead<std::string>(config["ros"], "ros_recv_packet_topic", ros_recv_packet_topic, "hesai_packets");
    pkt_sub_ = nh_->subscribe(ros_recv_packet_topic, 100, &SourceDriver::RecievePacket, this);
    driver_param.decoder_param.enable_udp_thread = false;
    subscription_spin_thread_ = new boost::thread(boost::bind(&SourceDriver::SpinRos1,this));
  }

  #ifdef __CUDACC__
    driver_ptr_.reset(new HesaiLidarSdkGpu<LidarPointXYZIRT>());
    driver_param.decoder_param.enable_parser_thread = false;
  #else
    driver_ptr_.reset(new HesaiLidarSdk<LidarPointXYZIRT>());
    driver_param.decoder_param.enable_parser_thread = true;
  #endif
  driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPointCloud, this, std::placeholders::_1));
  if(send_packet_ros && driver_param.input_param.source_type != DATA_FROM_ROS_PACKET){
    driver_ptr_->RegRecvCallback(std::bind(&SourceDriver::SendPacket, this, std::placeholders::_1)) ;
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

inline void SourceDriver::SendPacket(const UdpFrame_t& msg)
{
  pkt_pub_.publish(ToRosMsg(msg));
}

inline void SourceDriver::SendPointCloud(const LidarDecodedFrame<LidarPointXYZIRT>& msg)
{
  pub_.publish(ToRosMsg(msg, frame_id_));
}

inline sensor_msgs::PointCloud2 SourceDriver::ToRosMsg(const LidarDecodedFrame<LidarPointXYZIRT>& frame, const std::string& frame_id)
{
  sensor_msgs::PointCloud2 ros_msg;

  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);
  ros_msg.width = frame.points_num; 
  ros_msg.height = 1; 

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);

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
  printf("frame:%d points:%u packet:%d start time:%lf end time:%lf\n",frame.frame_index, frame.points_num, frame.packet_num, frame.points[0].timestamp, frame.points[frame.points_num - 1].timestamp) ;
  // ros_msg.header.seq = s;
  ros_msg.header.stamp = ros::Time().fromSec(frame.points[0].timestamp);
  ros_msg.header.frame_id = frame_id_;
  return ros_msg;
}

inline hesai_ros_driver::UdpFrame SourceDriver::ToRosMsg(const UdpFrame_t& ros_msg) {
  hesai_ros_driver::UdpFrame rs_msg;
  for (int i = 0 ; i < ros_msg.size(); i++) {
    hesai_ros_driver::UdpPacket rawpacket;
    rawpacket.size = ros_msg[i].packet_len;
    rawpacket.data.resize(ros_msg[i].packet_len);
    memcpy(&rawpacket.data[0], &ros_msg[i].buffer[0], ros_msg[i].packet_len);
    rs_msg.packets.push_back(rawpacket);
  }
  return rs_msg;
}

inline void SourceDriver::RecievePacket(const hesai_ros_driver::UdpFrame& msg)
{
  for (int i = 0; i < msg.packets.size(); i++) {
    driver_ptr_->lidar_ptr_->origin_packets_buffer_.emplace_back(&msg.packets[i].data[0], msg.packets[i].size);
  }
}


