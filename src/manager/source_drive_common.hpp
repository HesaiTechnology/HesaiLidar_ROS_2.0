#pragma once
#include "utility/yaml_reader.hpp"
#include "hesai_lidar_sdk.hpp"
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
        YamlRead<bool>(       driver_config, "use_gpu",                 driver_param.use_gpu, false);
        YamlRead<int>(        driver_config, "source_type",             source_type, 0);
        driver_param.input_param.source_type = SourceType(source_type);
        if (source_type == 1) {
            YamlRead<std::string>(driver_config["lidar_udp_type"], "device_ip_address",       driver_param.input_param.device_ip_address, "192.168.1.201");
            YamlRead<uint16_t>(   driver_config["lidar_udp_type"], "udp_port",                driver_param.input_param.udp_port, 2368);
            YamlRead<uint16_t>(   driver_config["lidar_udp_type"], "ptc_port",                driver_param.input_param.ptc_port, 9347);
            YamlRead<std::string>(driver_config["lidar_udp_type"], "group_address",           driver_param.input_param.multicast_ip_address, "");
            YamlRead<bool>(       driver_config["lidar_udp_type"], "use_ptc_connected",       driver_param.input_param.use_ptc_connected, true);
            YamlRead<std::string>(driver_config["lidar_udp_type"], "correction_file_path",    driver_param.input_param.correction_file_path, "");
            YamlRead<std::string>(driver_config["lidar_udp_type"], "firetimes_path",          driver_param.input_param.firetimes_path, "");
            YamlRead<bool>(       driver_config["lidar_udp_type"], "use_someip",              driver_param.input_param.use_someip, false);
            YamlRead<std::string>(driver_config["lidar_udp_type"], "host_ip_address",         driver_param.input_param.host_ip_address, "192.168.1.100");
            YamlRead<uint16_t>(   driver_config["lidar_udp_type"], "fault_message_port",      driver_param.input_param.fault_message_port, 0);
            YamlRead<int>(        driver_config["lidar_udp_type"], "standby_mode",            driver_param.input_param.standby_mode, -1);
            YamlRead<int>(        driver_config["lidar_udp_type"], "speed",                   driver_param.input_param.speed, -1);
        }
        else if (source_type == 2) {    
            YamlRead<std::string>(driver_config["pcap_type"], "pcap_path",                    driver_param.input_param.pcap_path, "");
            YamlRead<std::string>(driver_config["pcap_type"], "correction_file_path",         driver_param.input_param.correction_file_path, "");
            YamlRead<std::string>(driver_config["pcap_type"], "firetimes_path",               driver_param.input_param.firetimes_path, "");
            YamlRead<bool>(       driver_config["pcap_type"], "pcap_play_synchronization",    driver_param.decoder_param.pcap_play_synchronization, false);
            YamlRead<bool>(       driver_config["pcap_type"], "pcap_play_in_loop",            driver_param.decoder_param.pcap_play_in_loop, false);

        }
        else if (source_type == 4) {
            YamlRead<std::string>(driver_config["serial_type"], "rs485_com",                  driver_param.input_param.rs485_com, "/dev/ttyUSB0");
            YamlRead<std::string>(driver_config["serial_type"], "rs232_com",                  driver_param.input_param.rs232_com, "/dev/ttyUSB1");
            YamlRead<int>(        driver_config["serial_type"], "point_cloud_baudrate",       driver_param.input_param.point_cloud_baudrate, 3125000);
            YamlRead<std::string>(driver_config["serial_type"], "correction_save_path",       driver_param.input_param.correction_save_path, "");
            YamlRead<std::string>(driver_config["serial_type"], "correction_file_path",       driver_param.input_param.correction_file_path, "");
        }
        YamlRead<float>(      driver_config, "frame_start_azimuth",       driver_param.decoder_param.frame_start_azimuth, -1);
        YamlRead<uint16_t>(   driver_config, "use_timestamp_type",        driver_param.decoder_param.use_timestamp_type, 0);
        // decoder related
        YamlRead<bool>(       driver_config, "transform_flag",            driver_param.decoder_param.transform_param.use_flag, false);
        YamlRead<float>(      driver_config, "x",                         driver_param.decoder_param.transform_param.x, 0);
        YamlRead<float>(      driver_config, "y",                         driver_param.decoder_param.transform_param.y, 0);
        YamlRead<float>(      driver_config, "z",                         driver_param.decoder_param.transform_param.z, 0);
        YamlRead<float>(      driver_config, "roll",                      driver_param.decoder_param.transform_param.roll, 0);
        YamlRead<float>(      driver_config, "pitch",                     driver_param.decoder_param.transform_param.pitch, 0);
        YamlRead<float>(      driver_config, "yaw",                       driver_param.decoder_param.transform_param.yaw, 0);
        YamlRead<int>(        driver_config, "fov_start",                 driver_param.decoder_param.fov_start, -1);
        YamlRead<int>(        driver_config, "fov_end",                   driver_param.decoder_param.fov_end, -1);
        YamlRead<bool>(       driver_config, "enable_packet_loss_tool",   driver_param.decoder_param.enable_packet_loss_tool, false);
        YamlRead<bool>(       driver_config, "distance_correction_flag",  driver_param.decoder_param.distance_correction_flag, false);
        YamlRead<bool>(       driver_config, "xt_spot_correction",        driver_param.decoder_param.xt_spot_correction, false);
        YamlRead<uint16_t>(   driver_config, "device_udp_src_port",       driver_param.input_param.device_udp_src_port, 0);
        YamlRead<uint16_t>(   driver_config, "device_fault_port",         driver_param.input_param.device_fault_port, 0);
        
        // ROS related
        YamlRead<bool>(       config["ros"], "send_packet_ros",            driver_param.input_param.send_packet_ros, false);
        YamlRead<bool>(       config["ros"], "send_point_cloud_ros",       driver_param.input_param.send_point_cloud_ros, false);
        YamlRead<bool>(       config["ros"], "send_imu_ros",               driver_param.input_param.send_imu_ros, false);
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
        return true;
    }

    
};