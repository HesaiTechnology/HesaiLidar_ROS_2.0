/*
 * File: hesai_ros_driver_component.cc
 * Description: Hesai lidar driver as a ROS 2 composable node (rclcpp_components).
 */

#include "manager/hesai_ros_driver_component.h"

#include <fstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "Version.h"

namespace hesai_ros_driver
{

HesaiRosDriverNode::HesaiRosDriverNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("hesai_ros_driver_node", options)
{
  RCLCPP_INFO(get_logger(), "-------- Hesai Lidar ROS V%d.%d.%d --------",
              VERSION_MAJOR, VERSION_MINOR, VERSION_TINY);

  const std::string config_path = ResolveConfigPath();
  RCLCPP_INFO(get_logger(), "Loading config: %s", config_path.c_str());

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Load config error: %s", e.what());
    throw;
  }
  // Check here so a bad config does not hit exit() in YamlSubNodeAbort and take the container down.
  if (!config["lidar"]) {
    RCLCPP_FATAL(get_logger(), "Config '%s' has no 'lidar' section", config_path.c_str());
    throw std::runtime_error("Missing 'lidar' section in Hesai config");
  }

  node_manager_ = std::make_shared<NodeManager>();
  node_manager_->Init(config, this);
  node_manager_->Start();
}

HesaiRosDriverNode::~HesaiRosDriverNode()
{
  // Stop SDK threads before the node (and its publishers) are torn down.
  if (node_manager_) {
    node_manager_->Stop();
    node_manager_.reset();
  }
}

bool HesaiRosDriverNode::IsPlayEnded()
{
  return node_manager_ ? node_manager_->IsPlayEnded() : true;
}

std::string HesaiRosDriverNode::ResolveConfigPath()
{
  const std::string path = declare_parameter<std::string>("config_path", "");
  if (!path.empty()) {
    return path;
  }
  // Same default as the original executable: the config in the source tree.
  const std::string source_config = std::string(PROJECT_PATH) + "/config/config.yaml";
  if (std::ifstream(source_config).good()) {
    return source_config;
  }
  // Source tree not available (e.g. binary deployment): use the installed copy.
  try {
    return ament_index_cpp::get_package_share_directory("hesai_ros_driver") + "/config/config.yaml";
  } catch (const std::exception&) {
    return source_config;
  }
}

}  // namespace hesai_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(hesai_ros_driver::HesaiRosDriverNode)
