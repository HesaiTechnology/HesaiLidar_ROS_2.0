# HesaiLidar_ROS_2.0

## V2.0.12

### Monday April 27th, 2026 20:47:57

## Added
1. Added `ros_send_every_packet_topic` feature to publish every lidar packet one by one through ROS.
2. Added `ptp_status_size` field to the `Ptp` message and changed `ptp_status` from a fixed-length array to a variable-length array.
3. Added support for passing SDK CMake arguments during ROS1 and ROS2 build.
4. Added installation of the `config` directory for ROS2 install output.
5. Added configuration file load exception handling when starting the node.

## Modified
1. Updated GitHub Actions workflow to install required system dependencies and upload build logs correctly.
2. Updated the build workflow to be compatible with ROS Rolling compilation.
3. Updated `HesaiLidar_SDK_2.0` submodule to a newer commit.
4. Updated `CMakeLists.txt` to improve compatibility across ROS2 distributions and different `ament` / `rosidl` interfaces.
5. Updated correction and PTP topic publishing behavior to support late subscribers more reliably.
6. Updated IMU publisher queue size for PCAP and ROS packet input scenarios.
7. Updated IMU output publishing logic to use the raw driver values directly.
8. Updated ROS launch and configuration examples, including `config_path` and packet topic related settings.
9. Updated README with supported lidar models, build instructions, and SDK CMake option descriptions.
10. Simplified the default RViz configuration.
11. Removed `ros_send_firetime_topic` related configuration and publishing logic.

## V2.0.11

### Wednesday November 5th, 2025 16:30:00

## Added
1. Added play_rate_ feature to control PCAP playback rate, with a default value of 1.0 representing 1x playback speed.
2. Added channel_fov_filter_path feature to configure FOV files for filtering point cloud data with multiple FOVs under multiple channels.
3. Added multi_fov_filter_ranges feature to configure FOV files for filtering point cloud data with multiple FOVs across all channels.
4. Added frame_frequency feature to configure point cloud publishing frequency, requiring manual configuration of default_frame_frequency to the actual point cloud publishing frequency.
5. Added host_ptc_port feature to configure the local port when connecting to PTC.
6. Added echo_mode_filter feature to configure echo mode filtering, with a default value of 0 indicating no filtering.


## V2.0.10

### Monday June 30rd, 2025 17:28:15

### modify
1. Modify the units of IMU output data to m/s² and rad/s respectively
2. Use multicast_ip_address instead of group_address
3. Modify the config.yaml configuration table, configure different data sources through source_type, and fill in the corresponding configuration items
4. send_imu_ros controls whether IMU data registers callback functions and whether to publish IMU data
5. Support jazzy version, support install

## V2.0.9

### Monday December 23rd, 2024 14:57:29 CST

### modify
1. Distance Correction and XT S Point Cloud Layering Correction No need to specify radar model, via flag position switch

## V2.0.8

### Wednesday September 25th, 2024 11:04:33 CST

### modify
1. Addition of mechanical lidar photocentre correction parameters: distance_correction_lidar_type
2. Adding XT lidar point cloud S-stratification correction parameters: lidar_type
3. Add parameter to filter point cloud or fault message data for a specified port: device_udp_src_port, device_fault_port

## V2.0.7

### Saturday April 13th, 2024 20:10:00 CST

### modify
1. Fix gpu compile problem
2. Resolve compile warnings, inluding cpu compile and gpu compile
3. ROS add topics, support angle correction data, ptp data, loss bag count data

## V2.0.6

### Monday, January 15th, 2024 21:30

### modify
1. Add point cloud timestamp usage type configuration
2. Support P128 and XT32 to set the speed and standby through the configuration file
3. Add support for AT128 to intercept FOV display

## V2.0.5
### Wednesday, October 25th, 2023 20:00 

### modify
1. Fix bug in decode OT128 timestamp

## V2.0.4

### Monday, October 16th, 2023 11:00 

### modify
1. support ET25 

## V2.0.1

### Friday, June 9th, 2023 16:45 

### modify
1. first update
2. fix AT128 frame segmentation bug


