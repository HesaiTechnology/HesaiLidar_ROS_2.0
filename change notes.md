# HesaiLidar_ROS_2.0

################################################################################
Friday, June 9th, 2023 16:45 
## version
V2.0.1

## modify
1. first update
2. fix AT128 frame segmentation bug

################################################################################
Monday, October 16th, 2023 11:00 
## version
V2.0.4

## modify
1. support ET25 

################################################################################
Wednesday, October 25th, 2023 20:00 
## version
V2.0.5

## modify
1. Fix bug in decode OT128 timestamp

################################################################################
Monday, January 15th, 2024 21:30
## version
V2.0.6

## modify
1. Add point cloud timestamp usage type configuration
2. Support P128 and XT32 to set the speed and standby through the configuration file
3. Add support for AT128 to intercept FOV display

################################################################################
Saturday April 13th, 2024 20:10:00 CST
## version
V2.0.7

## modify
1. Fix gpu compile problem
2. Resolve compile warnings, inluding cpu compile and gpu compile
3. ROS add topics, support angle correction data, ptp data, loss bag count data

################################################################################
Wednesday September 25th, 2024 11:04:33 CST
## version
V2.0.8

## modify
1. Addition of mechanical lidar photocentre correction parameters: distance_correction_lidar_type
2. Adding XT lidar point cloud S-stratification correction parameters: lidar_type
3. Add parameter to filter point cloud or fault message data for a specified port: device_udp_src_port, device_fault_port

################################################################################
Monday December 23rd, 2024 14:57:29 CST
## version
V2.0.9

## modify
1. Distance Correction and XT S Point Cloud Layering Correction No need to specify radar model, via flag position switch

################################################################################
Monday June 30rd, 2025 17:28:15
## version
2.0.10

## modify
1. Modify the units of IMU output data to m/s² and rad/s respectively
2. Use multicast_ip_address instead of group_address
3. Modify the config.yaml configuration table, configure different data sources through source_type, and fill in the corresponding configuration items
4. send_imu_ros controls whether IMU data registers callback functions and whether to publish IMU data
5. Support jazzy version, support install
