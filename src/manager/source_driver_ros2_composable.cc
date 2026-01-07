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

HesaiComposableNode::HesaiComposableNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hesai_ros_driver_node", options)
{
  std::cout << "-------- Hesai Lidar ROS Composable Node --------" << std::endl;

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
      auto source = std::make_shared<SourceDriver>(SourceType::DATA_FROM_LIDAR);
      source->Init(lidar_config[i]);
      source->Start();
      sources_driver_.emplace_back(source);
    }
    
    RCLCPP_INFO(this->get_logger(), "Initialized %zu lidar source(s)", sources_driver_.size());
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
