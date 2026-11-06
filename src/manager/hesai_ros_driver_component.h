/*
 * File: hesai_ros_driver_component.h
 * Description: Hesai lidar driver as a ROS 2 composable node (rclcpp_components).
 */

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "manager/node_manager.h"

namespace hesai_ros_driver
{

class HesaiRosDriverNode : public rclcpp::Node
{
public:
  // Constructor signature required by rclcpp_components.
  // Parameters:
  //   config_path (string): path to the driver YAML config. Empty -> <share>/config/config.yaml
  explicit HesaiRosDriverNode(const rclcpp::NodeOptions& options);
  ~HesaiRosDriverNode() override;

  // True once every configured source has finished PCAP playback.
  // Has the same side effects as NodeManager::IsPlayEnded (used by the standalone executable).
  bool IsPlayEnded();

private:
  std::string ResolveConfigPath();

  std::shared_ptr<NodeManager> node_manager_;
};

}  // namespace hesai_ros_driver
