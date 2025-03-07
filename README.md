# Introduction to HesaiLidar_ROS_2.0
This repository includes the ROS Driver for Hesai LiDAR sensor manufactured by Hesai Technology. 
Developed based on [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0), After launched, the project will monitor UDP packets from Lidar,parse data and publish point cloud frames into ROS topic

## Support Lidar type
- Pandar
- AT128/AT128P
- QT
- FT120
- XT16/XT32
- ET25/ET30
- OT
- ATX
- JT16
- JT128、JT256 (need define JT128_256)

### Installation dependencies

Install ROS related dependency libraries, please refer to: http://wiki.ros.org
    
- Ubuntu 16.04 - ROS Kinetic desktop
- Ubuntu 18.04 - ROS Melodic desktop
- Ubuntu 20.04 - ROS Noetic desktop
- Ubuntu 18.04 - ROS2 Dashing desktop
- Ubuntu 20.04 - ROS2 Foxy desktop
- Ubuntu 22.04 - ROS2 Humble desktop

### Install Boost

    sudo apt-get update
    sudo apt-get install libboost-all-dev

### Install Yaml

    sudo apt-get update
    sudo apt-get install -y libyaml-cpp-dev

### Clone
```
$ git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
```    

### Compile and run

- ros1

    Create an `src` folder, copy the source code of the ros driver into it, and then run the following command:
        
        catkin_make
        source devel/setup.bash
        roslaunch hesai_ros_driver start.launch

- ros2

    Create an `src` folder, copy the source code of the ros driver into it, and then run the following command:
        
        colcon build --symlink-install
        . install/local_setup.bash

    For ROS2-Dashing     

        ros2 launch hesai_ros_driver dashing_start.py
        
    For other ROS2 version

        ros2 launch hesai_ros_driver start.py

### Introduction to the configuration file `config.yaml` parameters

    lidar:
        - driver:
            pcap_play_synchronization: true                       # pcap play rate synchronize with the host time
            udp_port: 2368                                        # UDP port of lidar
            ptc_port: 9347                                        # PTC port of lidar
            device_ip_address: 192.168.1.201                      # host_ip_address
            group_address: 255.255.255.255
            pcap_path: "<Your PCAP file path>"                    # The path of pcap file
            correction_file_path: "<Your correction file path>"   # The path of correction file
            firetimes_path: "<Your firetime file path>"           # The path of firetimes file
            source_type: 1                                        # The type of data source, 1: real-time lidar connection, 2: pcap, 3: packet rosbag, 4: serial    
            frame_start_azimuth: 0                                # Frame azimuth for Pandar128, range from 1 to 359, set it less than 0 if you do not want to use it
            #lidar configuration using serial communication
            rs485_com: "Your serial port name for receiving point cloud"  # if using JT16, Port to receive the point cloud
            rs232_com: "Your serial port name for sending cmd"            # if using JT16, Port to send cmd
            correction_save_path: ""                                      # if using JT16, turn on when you need to store angle calibration files(from lidar)
            # transform param
            x: 0
            y: 0
            z: 0
            roll: 0
            pitch: 0
            yaw: 0
            # decoder param
            use_timestamp_type: 0                                 # 0 use point cloud timestamp; 1 use receive timestamp
            # fov config, [fov_start, fov_end] range [1, 359], [-1, -1]means use default
            fov_start: -1
            fov_end:  -1
            enable_packet_loss_tool: true                         # enable the udp packet loss detection tool
            distance_correction_lidar_flag: false                 # set to true when optical centre correction needs to be turned on
            xt_spot_correction: false                             # Set to TRUE when XT S point cloud layering correction is required
            device_udp_src_port: 0                                # Filter point clouds for specified source ports in case of multiple lidar, setting >=1024
            device_fault_port: 0                                  # Filter fault message for specified source ports in case of multiple lidar, setting >=1024
        ros:
            ros_frame_id: hesai_lidar                            # Frame id of packet message and point cloud message
            ros_recv_packet_topic: /lidar_packets                # Topic used to receive lidar packets from ROS
            ros_send_packet_topic: /lidar_packets                # Topic used to send lidar packets through ROS
            ros_send_point_cloud_topic: /lidar_points            # Topic used to send point cloud through ROS
            ros_send_imu_topic: /lidar_imu                       # Topic used to send lidar imu message
            send_packet_ros: true                                # true: Send packets through ROS 
            send_point_cloud_ros: true                           # true: Send point cloud through ROS 

### Real time playback

Set the `source_type` in the configuration file to `1` and input the correct lidar `udp_port`, `ptc_port` (default 9347, usually unchanged) and `device_ip_address`, then run start.launch.

### Parsing PCAP file

Set the `source_type` in the configuration file to `2` and input the correct lidar `pcap_path` , `correction_file_path` and `firetime_file_path`, then run start.launch.

### Record and playback ROSBAG file

- Record ：

    When playing or parsing PCAP in real-time, set `send_packet_ros` to `true`, start another terminal and enter the following command to record the data packet ROSBAG.
        
        rosbag record ros_send_packet_topic

- Playback ：

    First, replay the recorded rosbag file `test.bag` using the following command.
        
        rosbag play test.bag

    Set the `source_type` in the configuration file to `3` and input the correct lidar `correction_file_path` , `firetime_file_path` and `ros_recv_packet_topic`(the topic name of rosbag), then run start.launch.

### Realize multi lidar fusion

According to the configuration of a single lidar, multiple drivers can be created in `config.yaml`, as shown in the following example

    lidar:
        - driver:              
            udp_port: 2368                  
            ptc_port: 9347              
            device_ip_address: 192.168.1.201          
            pcap_path: "<The PCAP file path>"                  
            correction_file_path: "<The correction file path>" 
            firetimes_path: "<Your firetime file path>"       
            source_type: 2          
            pcap_play_synchronization: true                   
            x: 0                                      
            y: 0                                     
            z: 0                                
            roll: 0                                 
            pitch: 0                             
            yaw: 0                                   
        ros:
            ros_frame_id: hesai_lidar                  
            ros_recv_packet_topic: /lidar_packets      
            ros_send_packet_topic: /lidar_packets      
            ros_send_point_cloud_topic: /lidar_points  
            send_packet_ros: true                     
            send_point_cloud_ros: true             
        - driver:               
            udp_port: 2368                         
            ptc_port: 9347                           
            device_ip_address: 192.168.1.201                  
            pcap_path: "<The PCAP file path>"                   
            correction_file_path: "<The correction file path>"  
            firetimes_path: "<Your firetime file path>"        
            source_type: 2        
            pcap_play_synchronization: true                     
            x: 0                                       
            y: 0                                       
            z: 0                                       
            roll: 0                                    
            pitch: 0                                   
            yaw: 0                                     
        ros:
            ros_frame_id: hesai_lidar                  
            ros_recv_packet_topic: /lidar_packets2     
            ros_send_packet_topic: /lidar_packets2     
            ros_send_point_cloud_topic: /lidar_points2 
            send_packet_ros: false                     
            send_point_cloud_ros: true                    
