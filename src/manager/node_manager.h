/************************************************************************************************
  Copyright(C)2023 Hesai Technology Co., Ltd.
  All code in this repository is released under the terms of the following [Modified BSD License.]
  Modified BSD License:
  Redistribution and use in source and binary forms,with or without modification,are permitted 
  provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice,this list of conditions 
   and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice,this list of conditions and 
   the following disclaimer in the documentation and/or other materials provided with the distribution.
  *Neither the names of the University of Texas at Austin,nor Austin Robot Technology,nor the names of 
   other contributors maybe used to endorse or promote products derived from this software without 
   specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGH THOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
  WARRANTIES,INCLUDING,BUT NOT LIMITED TO,THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
  PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT,INDIRECT,INCIDENTAL,SPECIAL,EXEMPLARY,OR CONSEQUENTIAL DAMAGES(INCLUDING,BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE,DATA,OR PROFITS;OR BUSINESS INTERRUPTION)HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY,WHETHER IN CONTRACT,STRICT LIABILITY,OR TORT(INCLUDING NEGLIGENCE 
  OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF 
  SUCHDAMAGE.
************************************************************************************************/

/*
 * File: node_manager.h
 * Author: Zhang Yu <zhangyu@hesaitech.com>
 * Description: ROS Node Manager
 * Created on June 12, 2023, 10:46 AM
 */

#pragma once

#include "utility/yaml_reader.hpp"
#ifdef ROS_FOUND
#include "source_driver_ros1.hpp"
#elif ROS2_FOUND
#include "source_driver_ros2.hpp"
#endif

class NodeManager
{
public:

  // Initialize ROS nodes based on configuration files
  void Init(const YAML::Node& config);
  // Start working
  void Start();
  // Stop working
  void Stop();
  bool IsPlayEnded();
  ~NodeManager();
  NodeManager() = default;
  std::vector<SourceDriver::Ptr> GetSourcesDriver();
private:

  std::vector<SourceDriver::Ptr> sources_driver_;
};


