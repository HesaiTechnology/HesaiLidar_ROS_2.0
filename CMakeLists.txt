cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)

#=======================================
# Version information
#=======================================
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  7)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
message(=============================================================)
message("-- ROS_VERSION is $ENV{ROS_VERSION}")
message(=============================================================)


#========================
# Project details / setup
#========================
set(PROJECT_NAME hesai_ros_driver)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
add_definitions(-std=c++17)
else()
add_definitions(-std=c++14)
endif()

add_compile_options(-Wall)

#========================
# Dependencies Setup
#========================

#ROS#
#Catkin#
if($ENV{ROS_VERSION} MATCHES "1")
  find_package(roscpp 1.12 QUIET)
  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})
  add_definitions(-DROS_FOUND)
  add_definitions(-DRUN_IN_ROS_WORKSPACE)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    roslib)

  add_message_files(
    FILES
    "UdpPacket.msg"
    "UdpFrame.msg"
    "LossPacket.msg"
    "Ptp.msg"
    "Firetime.msg"
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )


  catkin_package(CATKIN_DEPENDS 
    sensor_msgs 
    roslib)

endif($ENV{ROS_VERSION} MATCHES "1")

#ROS2#
if($ENV{ROS_VERSION} MATCHES "2")

  find_package(rclcpp QUIET)
  if(rclcpp_FOUND)

    message(=============================================================)
    message("-- ROS2 Found. ROS2 Support is turned On.")
    message(=============================================================)

    add_definitions(-DROS2_FOUND)
    include_directories(${rclcpp_INCLUDE_DIRS})
    set(CMAKE_CXX_STANDARD 14)

    find_package(ament_cmake REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)  
    find_package(rclcpp_action REQUIRED)
    find_package(rosidl_typesupport_c REQUIRED) 
    find_package(rosidl_default_generators REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs REQUIRED)
    
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/msg_ros2/UdpPacket.msg"
      "msg/msg_ros2/UdpFrame.msg"
      "msg/msg_ros2/Firetime.msg"
      "msg/msg_ros2/Ptp.msg"
      "msg/msg_ros2/LossPacket.msg"
      DEPENDENCIES builtin_interfaces std_msgs
    )
    ament_export_dependencies(rosidl_default_runtime)                 
    
  else(rclcpp_FOUND)

    message(=============================================================)
    message("-- ROS2 Not Found. ROS2 Support is turned Off.")
    message(=============================================================)

  endif(rclcpp_FOUND )
endif($ENV{ROS_VERSION} MATCHES "2")


#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)

#========================
# Build Setup
#========================

# add_executable(hesai_ros_driver_node
#                node/hesai_ros_driver_node.cpp
#                src/manager/node_manager.cpp
#                )
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# if install different cuda version, set the cuda path, like cuda-11.4
# set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.4/)
# find_package(CUDA REQUIRED)
find_package(CUDA )

if(CUDA_FOUND)
  
  message(=============================================================)
  message("-- CUDA Found. CUDA Support is turned On.")
  message(=============================================================)

  link_directories($ENV{CUDA_PATH}/lib/x64)
  set(CUDA_NVCC_FLAGS "-arch=sm_75;-O2")#根据具体GPU性能更改算力参数
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true")
  list(APPEND CUDA_NVCC_FLAGS -Xcompiler -fPIC)

  CUDA_ADD_EXECUTABLE(hesai_ros_driver_node
                node/hesai_ros_driver_node.cu
                src/manager/node_manager.cu
                ./src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/src/buffer.cu
                )
  set(CUDA_LIBS "${CUDA_TOOLKIT_ROOT_DIR}/lib64/libcudart.so")

  target_link_libraries(hesai_ros_driver_node                   
                ${YAML_CPP_LIBRARIES}
                ${Boost_LIBRARIES}
                source_lib
                container_lib
                ptcClient_lib
                ptcParser_lib
                log_lib
                ${CUDA_LIBS}
                # libhesai
    )
else(CUDA_FOUND)

  message(=============================================================)
  message("-- CUDA Not Found. CUDA Support is turned Off.")
  message(=============================================================)
  add_executable(hesai_ros_driver_node
                node/hesai_ros_driver_node.cc
                src/manager/node_manager.cc
                )
  target_link_libraries(hesai_ros_driver_node                   
              ${YAML_CPP_LIBRARIES}
              ${Boost_LIBRARIES}
              source_lib
              container_lib
              ptcClient_lib
              ptcParser_lib
              log_lib
              # libhesai
  )            

endif(CUDA_FOUND)


  target_include_directories(hesai_ros_driver_node PRIVATE
  src/driver/HesaiLidar_SDK_2.0/
  src/driver/HesaiLidar_SDK_2.0/libhesai
  src/driver/HesaiLidar_SDK_2.0/libhesai/Lidar
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParser/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpProtocol
  src/driver/HesaiLidar_SDK_2.0/libhesai/Source/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/Container/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/Container/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/UdpParserGpu/src
  src/driver/HesaiLidar_SDK_2.0/libhesai/PtcClient/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/PtcParser/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/PtcParser
  src/driver/HesaiLidar_SDK_2.0/libhesai/Logger/include
  src/driver/HesaiLidar_SDK_2.0/libhesai/include
  src/driver/HesaiLidar_SDK_2.0/driver
  src/manager
  src/msg/ros_msg
  src/msg/rs_msg
  src/utility
  ${CMAKE_CURRENT_BINARY_DIR}
)  

#Ros#
if($ENV{ROS_VERSION} MATCHES "1")
  target_link_libraries(hesai_ros_driver_node  ${ROS_LIBS})
  install(TARGETS hesai_ros_driver_node
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif($ENV{ROS_VERSION} MATCHES "1")


#Ros2#

if($ENV{ROS_VERSION} MATCHES "2")
  find_package(ament_index_cpp REQUIRED)
  ament_target_dependencies(hesai_ros_driver_node 
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp" 
  "rcutils"
  "std_msgs" 
  "sensor_msgs" 
  # "tf2_geometry_msgs"
  )
  rosidl_target_interfaces(hesai_ros_driver_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  install(TARGETS
    hesai_ros_driver_node
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})
    

  ament_package()

endif($ENV{ROS_VERSION} MATCHES "2")


