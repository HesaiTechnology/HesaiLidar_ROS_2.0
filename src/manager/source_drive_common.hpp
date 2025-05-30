#pragma once
#include "utility/yaml_reader.hpp"
#ifdef __CUDACC__
  #include "hesai_lidar_sdk_gpu.cuh"
#else
  #include "hesai_lidar_sdk.hpp"
#endif
class DriveYamlParam
{
public:
    DriveYamlParam() {};
    ~DriveYamlParam() {};

    bool GetDriveYamlParam(const YAML::Node& config, DriverParam &driver_param)
    {
        YAML::Node driver_config = YamlSubNodeAbort(config, "driver");
        int source_type;

        // input related
        YamlRead<uint16_t>(   driver_config, "udp_port",                driver_param.input_param.udp_port, 2368);
        YamlRead<uint16_t>(   driver_config, "ptc_port",                driver_param.input_param.ptc_port, 9347);
        YamlRead<std::string>(driver_config, "host_ip_address",         driver_param.input_param.host_ip_address, "192.168.1.100");
        YamlRead<std::string>(driver_config, "group_address",           driver_param.input_param.multicast_ip_address, "");
        YamlRead<std::string>(driver_config, "pcap_path",               driver_param.input_param.pcap_path, "");
        YamlRead<std::string>(driver_config, "firetimes_path",          driver_param.input_param.firetimes_path, "");
        YamlRead<std::string>(driver_config, "correction_file_path",    driver_param.input_param.correction_file_path, "");
        YamlRead<int>(        driver_config, "standby_mode",            driver_param.input_param.standby_mode, -1);
        YamlRead<int>(        driver_config, "speed",                   driver_param.input_param.speed, -1);
        YamlRead<std::string>(driver_config, "rs485_com",               driver_param.input_param.rs485_com, "/dev/ttyUSB0");
        YamlRead<std::string>(driver_config, "rs232_com",               driver_param.input_param.rs232_com, "/dev/ttyUSB1");
        YamlRead<std::string>(driver_config, "correction_save_path",    driver_param.input_param.correction_save_path, "");
        // decoder related
        YamlRead<bool>(       driver_config, "pcap_play_synchronization", driver_param.decoder_param.pcap_play_synchronization, false);
        YamlRead<float>(      driver_config, "x",                         driver_param.decoder_param.transform_param.x, 0);
        YamlRead<float>(      driver_config, "y",                         driver_param.decoder_param.transform_param.y, 0);
        YamlRead<float>(      driver_config, "z",                         driver_param.decoder_param.transform_param.z, 0);
        YamlRead<float>(      driver_config, "roll",                      driver_param.decoder_param.transform_param.roll, 0);
        YamlRead<float>(      driver_config, "pitch",                     driver_param.decoder_param.transform_param.pitch, 0);
        YamlRead<float>(      driver_config, "yaw",                       driver_param.decoder_param.transform_param.yaw, 0);
        YamlRead<std::string>(driver_config, "device_ip_address",         driver_param.input_param.device_ip_address, "192.168.1.201");
        YamlRead<float>(      driver_config, "frame_start_azimuth",       driver_param.decoder_param.frame_start_azimuth, -1);
        YamlRead<uint16_t>(   driver_config, "use_timestamp_type",        driver_param.decoder_param.use_timestamp_type, 0);
        YamlRead<int>(        driver_config, "fov_start",                 driver_param.decoder_param.fov_start, -1);
        YamlRead<int>(        driver_config, "fov_end",                   driver_param.decoder_param.fov_end, -1);
        YamlRead<int>(        driver_config, "source_type",               source_type, 0);
        YamlRead<bool>(       driver_config, "distance_correction_lidar_flag",  driver_param.decoder_param.distance_correction_lidar_flag, false);
        YamlRead<bool>(       driver_config, "xt_spot_correction",              driver_param.decoder_param.xt_spot_correction, false);
        YamlRead<std::string>(driver_config, "lidar_type",                      driver_param.lidar_type, "");
        YamlRead<uint16_t>(   driver_config, "device_udp_src_port",             driver_param.input_param.device_udp_src_port, 0);
        YamlRead<uint16_t>(   driver_config, "device_fault_port",               driver_param.input_param.device_fault_port, 0);
        driver_param.input_param.source_type = SourceType(source_type);
        // ROS related
        YamlRead<bool>(       driver_config, "enable_packet_loss_tool",    driver_param.decoder_param.enable_packet_loss_tool, false);
        YamlRead<bool>(       config["ros"], "send_packet_ros",            driver_param.input_param.send_packet_ros, false);
        YamlRead<bool>(       config["ros"], "send_point_cloud_ros",       driver_param.input_param.send_point_cloud_ros, false);
        YamlRead<std::string>(config["ros"], "ros_frame_id",               driver_param.input_param.frame_id, "hesai_lidar");
        YamlRead<std::string>(config["ros"], "ros_send_packet_topic",      driver_param.input_param.ros_send_packet_topic, "hesai_packets");
        YamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", driver_param.input_param.ros_send_point_topic, "hesai_points");
        YamlRead<std::string>(config["ros"], "ros_recv_packet_topic",      driver_param.input_param.ros_recv_packet_topic, "hesai_packets");
        YamlRead<std::string>(config["ros"], "ros_send_packet_loss_topic", driver_param.input_param.ros_send_packet_loss_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_send_ptp_topic",         driver_param.input_param.ros_send_ptp_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_send_correction_topic",  driver_param.input_param.ros_send_correction_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_send_firetime_topic",    driver_param.input_param.ros_send_firetime_topic, NULL_TOPIC);
        YamlRead<std::string>(config["ros"], "ros_recv_correction_topic",  driver_param.input_param.ros_recv_correction_topic, NULL_TOPIC);  
        YamlRead<std::string>(config["ros"], "ros_send_imu_topic",         driver_param.input_param.ros_send_imu_topic, NULL_TOPIC);
	YamlRead<float>(       config["ros"], "ros_imu_publish_rate",       driver_param.input_param.ros_imu_publish_rate, -1.0);	
	return true;
    }

    
};
