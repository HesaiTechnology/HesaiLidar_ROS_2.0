cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)

#=======================================
# Version information
#=======================================
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  11)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

set(WITH_PTCS_USE ON CACHE BOOL "Use Ptcs")
# set(FIND_CUDA ON CACHE BOOL "Find CUDA")

if(FIND_CUDA)
    find_package(CUDA )
endif()
if(${CUDA_FOUND})
  	project(CommonModules LANGUAGES CXX CUDA)
	set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
	set(CUDA_SEPARABLE_COMPILATION ON)
	include_directories(${CUDA_INCLUDE_DIRS})
	set(CUDA_PROPAGATE_HOST_FLAGS OFF)
	set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;)#根据具体GPU性能更改算力参数
	link_directories($ENV{CUDA_PATH}/lib/x64)

  add_definitions(-DUSE_CUDA)
  set(FIND_CUDA ON CACHE BOOL "Use Cuda")
else(${CUDA_FOUND})
	MESSAGE(STATUS "cuda not found!")
endif(${CUDA_FOUND})

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

if(($ENV{ROS_DISTRO} STREQUAL "kilted"))
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif(($ENV{ROS_DISTRO} STREQUAL "jazzy"))
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
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
set(DISENABLE_TEST_CC ON CACHE BOOL "DISENABLE_TEST_CC")  
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)

#========================
# Build Setup
#========================

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(LIB_TARGETS 
    hesai_sdk_lib
    log_lib
    lidar_lib
    ptcClient_lib
    source_lib
    platutils_lib
    serialClient_lib
    udpParser_lib
    )

if(${CUDA_FOUND})
  add_executable(hesai_ros_driver_node
    node/hesai_ros_driver_node.cu
    src/manager/node_manager.cc
    )
  set(LIB_TARGETS
    ${LIB_TARGETS}
    udpParserGpu_lib
    )
else()
  add_executable(hesai_ros_driver_node
    node/hesai_ros_driver_node.cc
    src/manager/node_manager.cc
    )
endif()
target_link_libraries(hesai_ros_driver_node                   
            ${YAML_CPP_LIBRARIES}
            ${Boost_LIBRARIES}
            ${LIB_TARGETS}
)              

target_include_directories(hesai_ros_driver_node PRIVATE
  src/manager
  src/msg/ros_msg
  src/msg/rs_msg
  src/utility
  ${CMAKE_CURRENT_BINARY_DIR}
)  

#Ros#
if($ENV{ROS_VERSION} MATCHES "1")
  target_link_libraries(hesai_ros_driver_node  ${ROS_LIBS})
  install(TARGETS hesai_ros_driver_node ${LIB_TARGETS}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  install(DIRECTORY
          launch
          rviz
          config
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
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

if(($ENV{ROS_DISTRO} STREQUAL "kilted") OR ($ENV{ROS_DISTRO} STREQUAL "jazzy") OR ($ENV{ROS_DISTRO} STREQUAL "humble"))
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(hesai_ros_driver_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(hesai_ros_driver_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

  install(TARGETS
          hesai_ros_driver_node
          DESTINATION lib/${PROJECT_NAME})
  install(TARGETS
          ${LIB_TARGETS}
          DESTINATION lib)

  install(DIRECTORY
          launch
          rviz
          DESTINATION share/${PROJECT_NAME})

  ament_package()

endif($ENV{ROS_VERSION} MATCHES "2")


