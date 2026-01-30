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
 * File: source_driver_ros2_composable.hpp
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Composable Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once
#include "source_driver_ros2.hpp"
#include <rclcpp_components/register_node_macro.hpp>

/**
 * @brief Structure to hold launch parameters that override config file values
 * 
 * These parameters are provided via ROS2 launch file instead of the YAML config
 */
struct LaunchOverrideParams
{
  std::string ros_frame_id;
  std::string ros_send_point_cloud_topic;
  std::string device_ip_address;
  uint16_t udp_port;
  uint16_t ptc_port;
  bool has_ros_frame_id = false;
  bool has_ros_send_point_cloud_topic = false;
  bool has_device_ip_address = false;
  bool has_udp_port = false;
  bool has_ptc_port = false;
};

class HesaiComposableNode : public rclcpp::Node
{
public:
  explicit HesaiComposableNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~HesaiComposableNode();

private:
  /**
   * @brief Declares and retrieves ROS2 parameters that override config file values
   * @return LaunchOverrideParams structure with the retrieved parameters
   */
  LaunchOverrideParams get_launch_override_params();

  std::vector<std::shared_ptr<SourceDriver>> sources_driver_;
};
