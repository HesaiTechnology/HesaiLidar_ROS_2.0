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
 * File: source_driver_ros2_composable.cc
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: Composable Source Driver for ROS2
 * Created on June 12, 2023, 10:46 AM
 */

#include "manager/source_driver_ros2_composable.hpp"
#include "utility/yaml_reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

LaunchOverrideParams HesaiComposableNode::get_launch_override_params()
{
  LaunchOverrideParams params;

  // Declare parameters with empty defaults - empty means use config file value
  this->declare_parameter<std::string>("ros_frame_id", "");
  this->declare_parameter<std::string>("ros_send_point_cloud_topic", "");
  this->declare_parameter<std::string>("device_ip_address", "");
  this->declare_parameter<int>("udp_port", 0);
  this->declare_parameter<int>("ptc_port", 0);

  // Retrieve parameters
  std::string ros_frame_id = this->get_parameter("ros_frame_id").as_string();
  std::string ros_send_point_cloud_topic = this->get_parameter("ros_send_point_cloud_topic").as_string();
  std::string device_ip_address = this->get_parameter("device_ip_address").as_string();
  int udp_port = this->get_parameter("udp_port").as_int();
  int ptc_port = this->get_parameter("ptc_port").as_int();

  // Check if parameters were provided (non-empty/non-zero means override)
  if (!ros_frame_id.empty()) {
    params.ros_frame_id = ros_frame_id;
    params.has_ros_frame_id = true;
    RCLCPP_INFO(this->get_logger(), "Using launch parameter ros_frame_id: %s", ros_frame_id.c_str());
  }

  if (!ros_send_point_cloud_topic.empty()) {
    params.ros_send_point_cloud_topic = ros_send_point_cloud_topic;
    params.has_ros_send_point_cloud_topic = true;
    RCLCPP_INFO(this->get_logger(), "Using launch parameter ros_send_point_cloud_topic: %s", ros_send_point_cloud_topic.c_str());
  }

  if (!device_ip_address.empty()) {
    params.device_ip_address = device_ip_address;
    params.has_device_ip_address = true;
    RCLCPP_INFO(this->get_logger(), "Using launch parameter device_ip_address: %s", device_ip_address.c_str());
  }

  if (udp_port > 0) {
    params.udp_port = static_cast<uint16_t>(udp_port);
    params.has_udp_port = true;
    RCLCPP_INFO(this->get_logger(), "Using launch parameter udp_port: %d", udp_port);
  }

  if (ptc_port > 0) {
    params.ptc_port = static_cast<uint16_t>(ptc_port);
    params.has_ptc_port = true;
    RCLCPP_INFO(this->get_logger(), "Using launch parameter ptc_port: %d", ptc_port);
  }

  return params;
}

HesaiComposableNode::HesaiComposableNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hesai_ros_driver_node", options)
{
  std::cout << "-------- Hesai Lidar ROS Composable Node --------" << std::endl;

  // Get launch override parameters first
  LaunchOverrideParams override_params = get_launch_override_params();

  std::string config_path;
  
  // Try to get config_path from parameter
  this->declare_parameter<std::string>("config_path", "");
  std::string path = this->get_parameter("config_path").as_string();
  
  if (!path.empty()) {
    config_path = path;
  } else {
    // Try PROJECT_PATH first (development environment)
#ifdef RUN_IN_ROS_WORKSPACE
    config_path = (std::string)PROJECT_PATH;
#else
    // Default to package share directory (installation)
    try {
      config_path = ament_index_cpp::get_package_share_directory("hesai_ros_driver");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get package share directory: %s", e.what());
      throw;
    }
#endif
    config_path += "/config/config.yaml";
  }

  RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_path.c_str());

  try {
    YAML::Node config = YAML::LoadFile(config_path);
    YAML::Node lidar_config = YamlSubNodeAbort(config, "lidar");
    
    for (uint8_t i = 0; i < lidar_config.size(); ++i) {
      YAML::Node single_lidar_config = lidar_config[i];
      
      // Apply launch parameter overrides to the YAML config
      // These parameters take precedence over config file values
      if (override_params.has_ros_frame_id) {
        single_lidar_config["ros"]["ros_frame_id"] = override_params.ros_frame_id;
      }
      if (override_params.has_ros_send_point_cloud_topic) {
        single_lidar_config["ros"]["ros_send_point_cloud_topic"] = override_params.ros_send_point_cloud_topic;
      }
      if (override_params.has_device_ip_address) {
        single_lidar_config["driver"]["lidar_udp_type"]["device_ip_address"] = override_params.device_ip_address;
      }
      if (override_params.has_udp_port) {
        single_lidar_config["driver"]["lidar_udp_type"]["udp_port"] = override_params.udp_port;
      }
      if (override_params.has_ptc_port) {
        single_lidar_config["driver"]["lidar_udp_type"]["ptc_port"] = override_params.ptc_port;
      }
      
      // Only initialize the first lidar (i=0) to avoid duplicate nodes
      // Multiple lidars in config should be handled by launching multiple instances
      if (i == 0) {
        auto source = std::make_shared<SourceDriver>(SourceType::DATA_FROM_LIDAR);
        // Set node_ptr_ to prevent SourceDriver::Init() from creating a new node
        // Use a custom deleter that does nothing to prevent deletion of this composable node
        source->node_ptr_ = std::shared_ptr<rclcpp::Node>(
          this, 
          [](rclcpp::Node*) {}  // Custom deleter - does nothing
        );
        source->Init(single_lidar_config);
        source->Start();
        sources_driver_.emplace_back(source);
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Initialized %zu lidar source(s)", sources_driver_.size());
    
    if (sources_driver_.size() == 0) {
      RCLCPP_WARN(this->get_logger(), "No lidar sources initialized. Check config file.");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Hesai driver: %s", e.what());
    throw;
  }
}

HesaiComposableNode::~HesaiComposableNode()
{
  for (auto& source : sources_driver_) {
    if (source) {
      source->Stop();
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(HesaiComposableNode)
