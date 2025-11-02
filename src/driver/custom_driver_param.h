// inherit from HesaiLidar_SDK_2.0/libhesai/driver_param.h
// then add car points filter options
/************************************************************************************************
 * Car points filter options
 ************************************************************************************************/
#pragma once
#include "HesaiLidar_SDK_2.0/libhesai/driver_param.h"

namespace hesai {
namespace lidar {

typedef struct CustomParam
{
  bool bubble_filter = false;
  double car_filter_distance = 0.0;
  bool cube_filter = false;
  double car_filter_distance_x = 0.0;
  double car_filter_distance_y = 0.0;
  double car_filter_distance_z = 0.0;
} CustomParam;

// New type that extends the SDK DriverParam with custom fields
struct CustomDriverParam : public hesai::lidar::DriverParam
{
  CustomParam custom_param;
};

} // namespace lidar
} // namespace hesai