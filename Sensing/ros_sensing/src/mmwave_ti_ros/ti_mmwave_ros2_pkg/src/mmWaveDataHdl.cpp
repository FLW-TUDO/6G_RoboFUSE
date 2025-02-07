/*
 * @file mmWaveDataHdl.cpp
 *
 * @brief
 * Creates the data handler node and sets parameters, with added socket functionality.
 *
 * 
 * NOTE:
 * (C) Copyright 2020 Texas Instruments, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_mmwave_ros2_pkg/mmWaveDataHdl.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ti_mmwave_ros2_pkg/DataHandlerClass.h"
#include <unistd.h>
#include <cstring>
#include <string>

namespace ti_mmwave_ros2_pkg {

mmWaveDataHdl::mmWaveDataHdl(const rclcpp::NodeOptions &options)
    : Node("mmWaveDataHdl", options) {
  onInit();
}

void mmWaveDataHdl::onInit() {
  std::string mySerialPort;
  std::string myFrameID;
  std::string ns;
  int myBaudRate;
  int myMaxAllowedElevationAngleDeg;
  int myMaxAllowedAzimuthAngleDeg;

  //ns = this->declare_parameter("namespace", "ep03");

  ns = this->declare_parameter("namespace", "");
    // if (!ns.empty()) {
    //     this->set_namespace(ns);
    // }
    


  mySerialPort = this->declare_parameter("data_port", "/dev/ttyUSB1");
  myBaudRate = this->declare_parameter("data_rate", 921600);
  myFrameID = this->declare_parameter("frame_id", "ti_mmwave_0");
  myMaxAllowedElevationAngleDeg =
      this->declare_parameter("max_allowed_elevation_angle_deg", 90);
  myMaxAllowedAzimuthAngleDeg =
      this->declare_parameter("max_allowed_azimuth_angle_deg", 90);

  // Socket parameters
  server_ip = this->declare_parameter("server_ip", "127.0.0.1");
  server_port = this->declare_parameter("server_port", 65432);

  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_port = %s",
              mySerialPort.c_str());
  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: data_rate = %d", myBaudRate);
  RCLCPP_INFO(this->get_logger(),
              "mmWaveDataHdl: max_allowed_elevation_angle_deg = %d",
              myMaxAllowedElevationAngleDeg);
  RCLCPP_INFO(this->get_logger(),
              "mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d",
              myMaxAllowedAzimuthAngleDeg);
  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: server_ip = %s",
              server_ip.c_str());
  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: server_port = %d",
              server_port);

  if (ns.compare("") != 0)
    ns = "/" + ns;
  
  RCLCPP_INFO(this->get_logger(), "Namespace: %s", this->get_namespace());
  auto DataUARTHandler_pub =
      create_publisher<PointCloud2>("/ep03/ti_mmwave/radar_scan_pcl", 100);
  auto radar_scan_pub =
      create_publisher<RadarScan>("/ep03/ti_mmwave/radar_scan", 100);
  auto marker_pub =
      create_publisher<Marker>("/ep03/ti_mmwave/radar_scan_markers", 100);

//   auto DataUARTHandler_pub =
//       create_publisher<PointCloud2>(ns + "/ti_mmwave/radar_scan_pcl", 100);
//   auto radar_scan_pub =
//       create_publisher<RadarScan>(ns + "/ti_mmwave/radar_scan", 100);
//   auto marker_pub =
//       create_publisher<Marker>(ns + "/ti_mmwave/radar_scan_markers", 100);

  DataHandler = std::make_shared<DataUARTHandler>();
  DataHandler->setNamespace(ns);
  DataHandler->onInit();
  DataHandler->setPublishers(DataUARTHandler_pub, radar_scan_pub, marker_pub);
  // Pass these parameters to DataUARTHandler
  DataHandler->setupSocket(server_ip, server_port);

  DataHandler->setFrameID((char *)myFrameID.c_str());
  DataHandler->setUARTPort((char *)mySerialPort.c_str());
  DataHandler->setBaudRate(myBaudRate);
  DataHandler->setMaxAllowedElevationAngleDeg(myMaxAllowedElevationAngleDeg);
  DataHandler->setMaxAllowedAzimuthAngleDeg(myMaxAllowedAzimuthAngleDeg);



  rclcpp::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(DataHandler);
  DataHandler->start();

  RCLCPP_INFO(this->get_logger(), "mmWaveDataHdl: Finished onInit function");
}
/*
mmWaveDataHdl::~mmWaveDataHdl() {
    // Clean up resources if necessary
    RCLCPP_INFO(this->get_logger(), "Destroying mmWaveDataHdl...");
}

*/

} // namespace ti_mmwave_ros2_pkg

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ti_mmwave_ros2_pkg::mmWaveDataHdl)
