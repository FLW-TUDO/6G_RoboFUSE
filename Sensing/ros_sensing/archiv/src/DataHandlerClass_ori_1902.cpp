/*
 * @file DataHandlerClass.cpp
 *
 * @brief
 * Handles and publishes incoming data from the sensor and .
 *
 * \par
 * NOTE:
 * (C) Copyright 2020 Texas Instruments, Inc.
 * ROS 2 Copyright 2022 Swimming Kim, Inc.
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

#include "ti_mmwave_ros2_pkg/DataHandlerClass.h"
#include <stdio.h>

//namespace fs = std::filesystem;


DataUARTHandler::DataUARTHandler()
    : rclcpp::Node("DataUARTHandler"), currentBufp(&pingPongBuffers[0]),
      nextBufp(&pingPongBuffers[1]) {
  //   DataUARTHandler_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
  //       "/ti_mmwave/radar_scan_pcl", 100);
  //   radar_scan_pub =
  //   create_publisher<ti_mmwave_ros2_interfaces::msg::RadarScan>(
  //       "/ti_mmwave/radar_scan", 100);
  //   marker_pub = create_publisher<visualization_msgs::msg::Marker>(
  //       "/ti_mmwave/radar_scan_markers", 100);
  // onInit();
}

void DataUARTHandler::onInit() {

  parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      this, ns + "/mmWaveCommSrvNode");

  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }

  auto parameters_future = parameters_client->get_parameters(
      {"numAdcSamples", "numLoops", "num_TX", "f_s", "f_c", "BW", "PRI", "t_fr",
       "max_range", "range_resolution", "max_doppler_vel",
       "doppler_vel_resolution"},
      std::bind(&DataUARTHandler::callbackGlobalParam, this,
                std::placeholders::_1));

  maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
  maxAllowedAzimuthAngleDeg = 90;   // Use max angle if none specified
}

void DataUARTHandler::setPublishers(
    const rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub_in)
    //const rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub_in,
    //const rclcpp::Publisher<Marker>::SharedPtr marker_pub_in) 
    
    {
  this->DataUARTHandler_pub = DataUARTHandler_pub_in;
  //this->radar_scan_pub = radar_scan_pub_in;
  ///this->marker_pub = marker_pub_in;
}

void DataUARTHandler::setNamespace(const std::string &ns) { this->ns = ns; }

void DataUARTHandler::callbackGlobalParam(
    std::shared_future<std::vector<rclcpp::Parameter>> future) {

  auto result = future.get();
  nr = result.at(0).as_int();
  nd = result.at(1).as_int();
  ntx = result.at(2).as_int();

  fs = static_cast<float>(result.at(3).as_double());
  fc = static_cast<float>(result.at(4).as_double());
  BW = static_cast<float>(result.at(5).as_double());
  PRI = static_cast<float>(result.at(6).as_double());
  tfr = static_cast<float>(result.at(7).as_double());
  max_range = static_cast<float>(result.at(8).as_double());
  vrange = static_cast<float>(result.at(9).as_double());
  max_vel = static_cast<float>(result.at(10).as_double());
  vvel = static_cast<float>(result.at(11).as_double());

  printf(
      "\n\n==============================\nList of "
      "parameters\n==============================\nNumber of range samples: "
      "%d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f "
      "MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange "
      "resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f "
      "m/s\n==============================\n",
      nr, nd, fs / 1e6, fc / 1e9, BW / 1e6, PRI * 1e6, tfr * 1e3, max_range,
      vrange, max_vel / 2, vvel);
}

void DataUARTHandler::setFrameID(char *myFrameID) { frameID = myFrameID; }

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char *mySerialPort) {
  dataSerialPort = mySerialPort;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int myBaudRate) { dataBaudRate = myBaudRate; }

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(
    int myMaxAllowedElevationAngleDeg) {
  maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(
    int myMaxAllowedAzimuthAngleDeg) {
  maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void) {

  int firstPacketReady = 0;
  uint8_t last8Bytes[8] = {0};

  /*Open UART Port and error checking*/
  serial::Serial mySerialObject("", dataBaudRate,
                                serial::Timeout::simpleTimeout(100));
  mySerialObject.setPort(dataSerialPort);
  try {
    mySerialObject.open();
  } catch (std::exception &e1) {
    printf("DataUARTHandler Read Thread: Failed to open Data serial port "
           "with error: %s",
           e1.what());
    printf("DataUARTHandler Read Thread: Waiting 20 "
           "seconds before trying again...");
    try {
      // Wait 20 seconds and try to open serial port again
      // ros::Duration(20).sleep();
      rclcpp::sleep_for(std::chrono::seconds(20));
      mySerialObject.open();
    } catch (std::exception &e2) {
      //   TODO
      //   RCLCPP_INFO(this->get_logger(),
      //               "DataUARTHandler Read Thread: Failed second time to open
      //               " "Data serial port, error: %s", e1.what());
      //   RCLCPP_INFO(this->get_logger(),
      //               "DataUARTHandler Read Thread: Port could not be opened.
      //               Port " "is \"%s\" and baud rate is %d", dataSerialPort,
      //               dataBaudRate);

      pthread_exit(NULL);
    }
  }

  if (mySerialObject.isOpen())
    printf("DataUARTHandler Read Thread: Port is open");
  else
    printf("DataUARTHandler Read Thread: Port could not be opened");

  /*Quick magicWord check to synchronize program with data Stream*/
  while (!isMagicWord(last8Bytes)) {

    last8Bytes[0] = last8Bytes[1];
    last8Bytes[1] = last8Bytes[2];
    last8Bytes[2] = last8Bytes[3];
    last8Bytes[3] = last8Bytes[4];
    last8Bytes[4] = last8Bytes[5];
    last8Bytes[5] = last8Bytes[6];
    last8Bytes[6] = last8Bytes[7];
    mySerialObject.read(&last8Bytes[7], 1);
  }

  /*Lock nextBufp before entering main loop*/
  pthread_mutex_lock(&nextBufp_mutex);

  while (rclcpp::ok()) {

    // std::cout << "readIncomingData" << std::endl;

    /*Start reading UART data and writing to buffer while also checking for
     * magicWord*/
    last8Bytes[0] = last8Bytes[1];
    last8Bytes[1] = last8Bytes[2];
    last8Bytes[2] = last8Bytes[3];
    last8Bytes[3] = last8Bytes[4];
    last8Bytes[4] = last8Bytes[5];
    last8Bytes[5] = last8Bytes[6];
    last8Bytes[6] = last8Bytes[7];
    mySerialObject.read(&last8Bytes[7], 1);

    nextBufp->push_back(last8Bytes[7]); // push byte onto buffer
	// DEBUG: Log bytes being read
//printf("Byte read from UART: %02x\n", last8Bytes[7]);
	
// Check if enough bytes have been accumulated to form a packet
if (nextBufp->size() >= PACKET_SIZE_THRESHOLD) {
    // Lock the sendQueue mutex before modifying shared resources
    if (pthread_mutex_lock(&sendQueueMutex) == 0) { // Lock POSIX mutex

        sendQueue.push(*nextBufp);      // Add buffer content as a new packet in queue
        nextBufp->clear();             // Clear buffer after pushing into queue

        pthread_cond_signal(&sendQueueCV); // Notify sender thread about new data

        pthread_mutex_unlock(&sendQueueMutex); // Unlock POSIX mutex after operation

    } else {
        printf("Error locking sendQueueMutex\n");
    }
}
     
    // ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x
    // %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5],
    // last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1],
    // last8Bytes[0]);

    /*If a magicWord is found wait for sorting to finish and switch buffers*/
    if (isMagicWord(last8Bytes)) {
      // ROS_INFO("Found magic word");

      /*Lock countSync Mutex while unlocking nextBufp so that the swap thread
       * can use it*/
      pthread_mutex_lock(&countSync_mutex);
      pthread_mutex_unlock(&nextBufp_mutex);

      /*increment countSync*/
      countSync++;

      /*If this is the first packet to be found, increment countSync again since
       * Sort thread is not reading data yet*/
      if (firstPacketReady == 0) {
        countSync++;
        firstPacketReady = 1;
      }

      /*Signal Swap Thread to run if countSync has reached its max value*/
      if (countSync == COUNT_SYNC_MAX) {
        pthread_cond_signal(&countSync_max_cv);
      }

      /*Wait for the Swap thread to finish swapping pointers and signal us to
       * continue*/
      pthread_cond_wait(&read_go_cv, &countSync_mutex);

      /*Unlock countSync so that Swap Thread can use it*/
      pthread_mutex_unlock(&countSync_mutex);
      pthread_mutex_lock(&nextBufp_mutex);

      nextBufp->clear();
      memset(last8Bytes, 0, sizeof(last8Bytes));
    }
  }

  mySerialObject.close();

  pthread_exit(NULL);
}

int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8]) {
  int val = 0, i = 0, j = 0;

  for (i = 0; i < 8; i++) {

    if (last8Bytes[i] == magicWord[i]) {
      j++;
    }
  }

  if (j == 8) {
    val = 1;
  }

  return val;
}

void *DataUARTHandler::syncedBufferSwap(void) {
  while (rclcpp::ok()) {

    // std::cout << "syncedBufferSwap" << std::endl;

    pthread_mutex_lock(&countSync_mutex);

    while (countSync < COUNT_SYNC_MAX) {
      pthread_cond_wait(&countSync_max_cv, &countSync_mutex);

      pthread_mutex_lock(&currentBufp_mutex);
      pthread_mutex_lock(&nextBufp_mutex);

      std::vector<uint8_t> *tempBufp = currentBufp;

      this->currentBufp = this->nextBufp;

      this->nextBufp = tempBufp;

      pthread_mutex_unlock(&currentBufp_mutex);
      pthread_mutex_unlock(&nextBufp_mutex);

      countSync = 0;

      pthread_cond_signal(&sort_go_cv);
      pthread_cond_signal(&read_go_cv);
    }

    pthread_mutex_unlock(&countSync_mutex);
  }

  pthread_exit(NULL);
}

void *DataUARTHandler::sortIncomingData(void) {
  MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
  uint32_t tlvLen = 0;
  uint32_t headerSize;
  unsigned int currentDatap = 0;
  SorterState sorterState = READ_HEADER;
  uint i = 0, tlvCount = 0; //, offset = 0;
  int j = 0;
  float maxElevationAngleRatioSquared;
  float maxAzimuthAngleRatio;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(
      new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::msg::PointCloud2 output_pointcloud;
  ti_mmwave_ros2_interfaces::msg::RadarScan radarscan;
  ParsedRadarPacket radarPacket; // Temporary storage for current radar frame

  // wait for first packet to arrive
  pthread_mutex_lock(&countSync_mutex);
  pthread_cond_wait(&sort_go_cv, &countSync_mutex);
  pthread_mutex_unlock(&countSync_mutex);

  pthread_mutex_lock(&currentBufp_mutex);

  while (rclcpp::ok()) {

    // std::cout << "sortIncomingData" << std::endl;

    switch (sorterState) {

    case READ_HEADER:

      // init variables
      mmwData.numObjOut = 0;

      // make sure packet has at least first three fields (12 bytes) before we
      // read them (does not include magicWord since it was already removed)
      if (currentBufp->size() < 12) {
        sorterState = SWAP_BUFFERS;
        break;
      }

      // get version (4 bytes)
      memcpy(&mmwData.header.version, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.version));
      currentDatap += (sizeof(mmwData.header.version));

      // get totalPacketLen (4 bytes)
      memcpy(&mmwData.header.totalPacketLen, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.totalPacketLen));
      currentDatap += (sizeof(mmwData.header.totalPacketLen));

      // get platform (4 bytes)
      memcpy(&mmwData.header.platform, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.platform));
      currentDatap += (sizeof(mmwData.header.platform));

      // if packet doesn't have correct header size (which is based on
      // platform), throw it away
      //  (does not include magicWord since it was already removed)
      if ((mmwData.header.platform & 0xFFFF) == 0x1443) // platform is xWR1443)
      {
        headerSize =
            7 * 4; // xWR1443 SDK demo header does not have subFrameNumber field
      } else {
        headerSize = 8 * 4; // header includes subFrameNumber field
      }
      if (currentBufp->size() < headerSize) {
        sorterState = SWAP_BUFFERS;
        break;
      }

      // get frameNumber (4 bytes)
      memcpy(&mmwData.header.frameNumber, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.frameNumber));
      currentDatap += (sizeof(mmwData.header.frameNumber));

      // get timeCpuCycles (4 bytes)
      memcpy(&mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.timeCpuCycles));
      currentDatap += (sizeof(mmwData.header.timeCpuCycles));

      // get numDetectedObj (4 bytes)
      memcpy(&mmwData.header.numDetectedObj, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.numDetectedObj));
      currentDatap += (sizeof(mmwData.header.numDetectedObj));

      // get numTLVs (4 bytes)
      memcpy(&mmwData.header.numTLVs, &currentBufp->at(currentDatap),
             sizeof(mmwData.header.numTLVs));
      currentDatap += (sizeof(mmwData.header.numTLVs));

      // get subFrameNumber (4 bytes) (not used for XWR1443)
      if ((mmwData.header.platform & 0xFFFF) != 0x1443) {
        memcpy(&mmwData.header.subFrameNumber, &currentBufp->at(currentDatap),
               sizeof(mmwData.header.subFrameNumber));
        currentDatap += (sizeof(mmwData.header.subFrameNumber));
      }

      // if packet lengths do not match, throw it away
      if (mmwData.header.totalPacketLen == currentBufp->size()) {
        sorterState = CHECK_TLV_TYPE;
      } else
        sorterState = SWAP_BUFFERS;

      break;

    case READ_OBJ_STRUCT:

      // CHECK_TLV_TYPE code has already read tlvType and tlvLen
      radarPacket.timestamp = rclcpp::Clock().now().seconds(); // Store timestamp in seconds
      i = 0;
      // offset = 0;

      if (((mmwData.header.version >> 24) & 0xFF) <
          3) // SDK version is older than 3.x
      {
        // get number of objects
        memcpy(&mmwData.numObjOut, &currentBufp->at(currentDatap),
               sizeof(mmwData.numObjOut));
        currentDatap += (sizeof(mmwData.numObjOut));

        // get xyzQFormat
        memcpy(&mmwData.xyzQFormat, &currentBufp->at(currentDatap),
               sizeof(mmwData.xyzQFormat));
        currentDatap += (sizeof(mmwData.xyzQFormat));
      } else // SDK version is at least 3.x
      {
        mmwData.numObjOut = mmwData.header.numDetectedObj;
      }

      // radarPacket.frameNumber = frameID;
      radarPacket.numObjects = mmwData.numObjOut;
      radarPacket.objects.clear(); 
      radarPacket.rangeProfile.clear(); 
      // RScan->header.seq = 0;
      // RScan->header.stamp = rclcpp::Clock().now();
      // RScan->header.stamp = (uint32_t) mmwData.header.timeCpuCycles;
      RScan->header.frame_id = frameID;
      RScan->height = 1;
      RScan->width = mmwData.numObjOut;
      RScan->is_dense = 1;
      RScan->points.resize(RScan->width * RScan->height);

      // Calculate ratios for max desired elevation and azimuth angles
      if ((maxAllowedElevationAngleDeg >= 0) &&
          (maxAllowedElevationAngleDeg < 90)) {
        maxElevationAngleRatioSquared =
            tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
        maxElevationAngleRatioSquared =
            maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
      } else
        maxElevationAngleRatioSquared = -1;
      if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90))
        maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
      else
        maxAzimuthAngleRatio = -1;

      // ROS_INFO("maxElevationAngleRatioSquared = %f",
      // maxElevationAngleRatioSquared); ROS_INFO("maxAzimuthAngleRatio = %f",
      // maxAzimuthAngleRatio); ROS_INFO("mmwData.numObjOut before = %d",
      // mmwData.numObjOut);

      // Populate pointcloud
      while (i < mmwData.numObjOut) {
        if (((mmwData.header.version >> 24) & 0xFF) <
            3) { // SDK version is older than 3.x
          
          // get object range index
          memcpy(&mmwData.objOut.rangeIdx, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.rangeIdx));
          currentDatap += (sizeof(mmwData.objOut.rangeIdx));

          // get object doppler index
          memcpy(&mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.dopplerIdx));
          currentDatap += (sizeof(mmwData.objOut.dopplerIdx));

          // get object peak intensity value
          memcpy(&mmwData.objOut.peakVal, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.peakVal));
          currentDatap += (sizeof(mmwData.objOut.peakVal));

          // get object x-coordinate
          memcpy(&mmwData.objOut.x, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.x));
          currentDatap += (sizeof(mmwData.objOut.x));

          // get object y-coordinate
          memcpy(&mmwData.objOut.y, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.y));
          currentDatap += (sizeof(mmwData.objOut.y));

          // get object z-coordinate
          memcpy(&mmwData.objOut.z, &currentBufp->at(currentDatap),
                 sizeof(mmwData.objOut.z));
          currentDatap += (sizeof(mmwData.objOut.z));

          float temp[7];

          temp[0] = (float)mmwData.objOut.x;
          temp[1] = (float)mmwData.objOut.y;
          temp[2] = (float)mmwData.objOut.z;
          temp[3] = (float)mmwData.objOut.dopplerIdx;

          for (int j = 0; j < 4; j++) {
            if (temp[j] > 32767)
              temp[j] -= 65536;
            if (j < 3)
              temp[j] = temp[j] / pow(2, mmwData.xyzQFormat);
          }

          temp[7] = temp[3] * vvel;

          temp[4] = (float)mmwData.objOut.rangeIdx * vrange;
          temp[5] = 10 * log10(mmwData.objOut.peakVal + 1); // intensity
          temp[6] = std::atan2(-temp[0], temp[1]) / M_PI * 180;

          uint16_t tmp = (uint16_t)(temp[3] + nd / 2);

          // Map mmWave sensor coordinates to ROS coordinate system
          RScan->points[i].x =
              temp[1]; // ROS standard coordinate system X-axis is forward which
                       // is the mmWave sensor Y-axis
          RScan->points[i].y =
              -temp[0]; // ROS standard coordinate system Y-axis is left which
                        // is the mmWave sensor -(X-axis)
          RScan->points[i].z =
              temp[2]; // ROS standard coordinate system Z-axis is up which is
                       // the same as mmWave sensor Z-axis
          RScan->points[i].intensity = temp[5];

          radarscan.header.frame_id = frameID;
          radarscan.header.stamp = rclcpp::Clock().now();

          radarscan.point_id = i;
          radarscan.x = temp[1];
          radarscan.y = -temp[0];
          radarscan.z = temp[2];
          radarscan.range = temp[4];
          radarscan.velocity = temp[7];
          radarscan.doppler_bin = tmp;
          radarscan.bearing = temp[6];
          radarscan.intensity = temp[5];
        } else { // SDK version is 3.x+
        // printf("SDK version is 3.x");
          // For socket data transfer
          ParsedRadarObject radarObj;
          // get object x-coordinate (meters)
          memcpy(&mmwData.newObjOut.x, &currentBufp->at(currentDatap),
                 sizeof(mmwData.newObjOut.x));
          currentDatap += (sizeof(mmwData.newObjOut.x));

          // get object y-coordinate (meters)
          memcpy(&mmwData.newObjOut.y, &currentBufp->at(currentDatap),
                 sizeof(mmwData.newObjOut.y));
          currentDatap += (sizeof(mmwData.newObjOut.y));

          // get object z-coordinate (meters)
          memcpy(&mmwData.newObjOut.z, &currentBufp->at(currentDatap),
                 sizeof(mmwData.newObjOut.z));
          currentDatap += (sizeof(mmwData.newObjOut.z));

          // get object velocity (m/s)
          memcpy(&mmwData.newObjOut.velocity, &currentBufp->at(currentDatap),
                 sizeof(mmwData.newObjOut.velocity));
          currentDatap += (sizeof(mmwData.newObjOut.velocity));

          // Calculate range from x, y, z
          radarObj.range =
              std::sqrt((mmwData.newObjOut.x * mmwData.newObjOut.x) +
                        (mmwData.newObjOut.y * mmwData.newObjOut.y) +
                        (mmwData.newObjOut.z * mmwData.newObjOut.z));
          
          // Calculate azimuth angle from x and y
          if (mmwData.newObjOut.y == 0) {
              radarObj.azimuth = (mmwData.newObjOut.x >= 0 ? 
                                  90.0f : -90.0f); 
          } else {
              radarObj.azimuth =
                  std::atan2(mmwData.newObjOut.x, mmwData.newObjOut.y) *
                  (180.0f / M_PI);   // Convert radians to degrees
          }

          // Calculate elevation angle from x, y, z
          if (mmwData.newObjOut.x == 0 && mmwData.newObjOut.y == 0) {
              radarObj.elevation = (mmwData.newObjOut.z >= 0 ? 90.0f : -90.0f);
          } else {
              radarObj.elevation =
                  std::atan2(mmwData.newObjOut.z,
                              std::sqrt((mmwData.newObjOut.x * mmwData.newObjOut.x) +
                                        (mmwData.newObjOut.y * mmwData.newObjOut.y))) *
                  (180.0f / M_PI); // Convert radians to degrees
          }

          // Map mmWave sensor coordinates to ROS coordinate system
          RScan->points[i].x =
              mmwData.newObjOut.y; // ROS standard coordinate system X-axis is
                                   // forward which is the mmWave sensor Y-axis
          RScan->points[i].y =
              -mmwData.newObjOut.x; // ROS standard coordinate system Y-axis is
                                    // left which is the mmWave sensor -(X-axis)
          RScan->points[i].z =
              mmwData.newObjOut
                  .z; // ROS standard coordinate system Z-axis is up which is
                      // the same as mmWave sensor Z-axis

          radarscan.header.frame_id = frameID;
          radarscan.header.stamp = rclcpp::Clock().now();

          radarscan.point_id = i;
          radarscan.x = mmwData.newObjOut.y;
          radarscan.y = -mmwData.newObjOut.x;
          radarscan.z = mmwData.newObjOut.z;
          // radarscan.range = temp[4];
          radarscan.velocity = mmwData.newObjOut.velocity;
          // radarscan.doppler_bin = tmp;
          // radarscan.bearing = temp[6];
          // radarscan.intensity = temp[5];


          radarObj.point_id = i;
          radarObj.x = mmwData.newObjOut.x;
          radarObj.y = -mmwData.newObjOut.y;
          radarObj.z = mmwData.newObjOut.z;
          // radarscan.range = temp[4];
          radarObj.velocity = mmwData.newObjOut.velocity;

          // Intensity will be updated later in READ_SIDE_INFO
          radarObj.intensity = 0.0f;
          // Add parsed object to radar packet
          radarPacket.objects.push_back(radarObj);
          // For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed
          // in the READ_SIDE_INFO code
        }
        /*
        if (((maxElevationAngleRatioSquared == -1) ||
             (((RScan->points[i].z * RScan->points[i].z) /
               (RScan->points[i].x * RScan->points[i].x +
                RScan->points[i].y * RScan->points[i].y)) <
              maxElevationAngleRatioSquared)) &&
            ((maxAzimuthAngleRatio == -1) ||
             (fabs(RScan->points[i].y / RScan->points[i].x) <
              maxAzimuthAngleRatio)) &&
            (RScan->points[i].x != 0)) {
          radar_scan_pub->publish(radarscan);
        } */
        i++;
      }

      sorterState = CHECK_TLV_TYPE;

      break;

    case READ_SIDE_INFO:

      // Make sure we already received and parsed detected obj list
      // (READ_OBJ_STRUCT)
      if (mmwData.numObjOut > 0) {
        for (i = 0; i < mmwData.numObjOut; i++) {
          // get snr (unit is 0.1 steps of dB)
          memcpy(&mmwData.sideInfo.snr, &currentBufp->at(currentDatap),
                 sizeof(mmwData.sideInfo.snr));
          currentDatap += (sizeof(mmwData.sideInfo.snr));

          // get noise (unit is 0.1 steps of dB)
          memcpy(&mmwData.sideInfo.noise, &currentBufp->at(currentDatap),
                 sizeof(mmwData.sideInfo.noise));
          currentDatap += (sizeof(mmwData.sideInfo.noise));

          RScan->points[i].intensity =
              (float)mmwData.sideInfo.snr /
              10.0; // Use snr for "intensity" field (divide by 10 since unit of
                    // snr is 0.1dB)

          // Update intensity field in ParsedRadarObject
            if (i < radarPacket.objects.size()) {
                radarPacket.objects[i].intensity = (float)mmwData.sideInfo.snr / 10.0;
            }
         
        }
      } else // else just skip side info section if we have not already received
             // and parsed detected obj list
      {
        i = 0;

        while (i++ < tlvLen - 1) {
          // ROS_INFO("DataUARTHandler Sort Thread : Parsing Side Info i=%d and
          // tlvLen = %u", i, tlvLen);
        }

        currentDatap += tlvLen;
      }

      sorterState = CHECK_TLV_TYPE;

      break;

    case READ_LOG_MAG_RANGE:
      // Only check for valid TLV length
      if (tlvLen > 0 && mmwData.numObjOut > 0) {
          size_t numBins = tlvLen / sizeof(int16_t);
          
          // Clear any previous range profile data
          radarPacket.rangeProfile.clear();

          // Bounds check
          if (currentDatap + tlvLen > currentBufp->size()) {
              printf("Error: Buffer overflow would occur processing range profile\n");
              currentDatap += tlvLen;
              break;
          }

          // Process range profile data
          for (size_t i = 0; i < numBins; ++i) {
              // Bounds check for each read
              if (currentDatap + sizeof(int16_t) <= currentBufp->size()) {
                  int16_t rawValue;
                  memcpy(&rawValue, &currentBufp->at(currentDatap), sizeof(rawValue));
                  currentDatap += sizeof(rawValue);

                  // Use configured number of range bins instead of hard-coded value
                  float dbValue = rpToDb(static_cast<float>(rawValue), nr); // nr is the configured range bins
                  float roundedValue = std::round(dbValue * 1000.0f) / 1000.0f;
                  radarPacket.rangeProfile.push_back(roundedValue);
              } else {
                  printf("Error: Incomplete range profile data\n");
                  break;
              }
          }

          if (radarPacket.rangeProfile.size() != numBins) {
              printf("Warning: Expected %zu range bins but processed %zu\n", 
                     numBins, radarPacket.rangeProfile.size());
          }
      } else {
          //printf("Invalid TLV length for Range Profile: %u\n", tlvLen);
          currentDatap += tlvLen;
      }
      
      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_NOISE:

      i = 0;

      while (i++ < tlvLen - 1) {
        // ROS_INFO("DataUARTHandler Sort Thread : Parsing Noise Profile i=%d
        // and tlvLen = %u", i, tlvLen);
      }

      currentDatap += tlvLen;

      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_AZIMUTH:

      i = 0;

      while (i++ < tlvLen - 1) {
        // ROS_INFO("DataUARTHandler Sort Thread : Parsing Azimuth Profile i=%d
        // and tlvLen = %u", i, tlvLen);
      }

      currentDatap += tlvLen;

      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_DOPPLER:

      i = 0;

      while (i++ < tlvLen - 1) {
        // ROS_INFO("DataUARTHandler Sort Thread : Parsing Doppler Profile i=%d
        // and tlvLen = %u", i, tlvLen);
      }

      currentDatap += tlvLen;

      sorterState = CHECK_TLV_TYPE;
      break;

    case READ_STATS:

      i = 0;

      while (i++ < tlvLen - 1) {
        // ROS_INFO("DataUARTHandler Sort Thread : Parsing Stats Profile i=%d
        // and tlvLen = %u", i, tlvLen);
      }

      currentDatap += tlvLen;

      sorterState = CHECK_TLV_TYPE;
      break;

    case CHECK_TLV_TYPE:

      // ROS_INFO("DataUARTHandler Sort Thread : tlvCount = %d, numTLV = %d",
      // tlvCount, mmwData.header.numTLVs);

      if (tlvCount++ >=
          mmwData.header.numTLVs) // Done parsing all received TLV sections
      {
        // Publish detected object pointcloud


        
        if (mmwData.numObjOut > 0) {
          j = 0;
          for (i = 0; i < mmwData.numObjOut; i++) {
            // Keep point if elevation and azimuth angles are less than
            // specified max values (NOTE: The following calculations are done
            // using ROS standard coordinate system axis definitions where X is
            // forward and Y is left)
            if (((maxElevationAngleRatioSquared == -1) ||
                 (((RScan->points[i].z * RScan->points[i].z) /
                   (RScan->points[i].x * RScan->points[i].x +
                    RScan->points[i].y * RScan->points[i].y)) <
                  maxElevationAngleRatioSquared)) &&
                ((maxAzimuthAngleRatio == -1) ||
                 (fabs(RScan->points[i].y / RScan->points[i].x) <
                  maxAzimuthAngleRatio)) &&
                (RScan->points[i].x != 0)) {
              // ROS_INFO("Kept point");
              // copy: points[i] => points[j]
              memcpy(&RScan->points[j], &RScan->points[i],
                     sizeof(RScan->points[i]));
              j++;
            }
          }
          mmwData.numObjOut = j; // update number of objects as some points may
                                 // have been removed

          // Resize point cloud since some points may have been removed
          RScan->width = mmwData.numObjOut;
          RScan->points.resize(RScan->width * RScan->height);

          // ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
          // ROS_INFO("DataUARTHandler Sort Thread: number of obj = %d",
          // mmwData.numObjOut );
          pcl::PCLPointCloud2 cloud_ROI;
          pcl::toPCLPointCloud2(*RScan, cloud_ROI);
          pcl_conversions::fromPCL(cloud_ROI, output_pointcloud);
          DataUARTHandler_pub->publish(output_pointcloud);
          
          // Serialize and send completed packet after processing side info
          std::string serializedJSONString = serializeRadarPacket(radarPacket);
          
          
          //printParsedRadarPacket(radarPacket);
          // Save to log file
          saveDataToLog(serializedJSONString);
          //printf("Timestamp: %f\n", mmwData.time);
          printf("Number of objects detected: %d\n", mmwData.numObjOut);
          // Send JSON over socket
          /*
          try {
              sendParsedDataOverSocket(serializedJSONString);
           
          } catch (const std::exception &e) {
              fprintf(stderr, "Error sending parsed radar data: %s\n", e.what());
          } */
          // std::vector<uint8_t> serializedBuffer = serializeMmwDataPacket_custom(mmwData);
          // sendRawDataOverSocket(serializedBuffer);
        }
        // Clear packet for next frame
        radarPacket.objects.clear();
        radarPacket.rangeProfile.clear();
        // ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says
        // tlvCount max was reached, going to switch buffer state");
        sorterState = SWAP_BUFFERS;
      }

      else // More TLV sections to parse
      {
        // get tlvType (32 bits) & remove from queue
        memcpy(&tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
        currentDatap += (sizeof(tlvType));

        // ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvType) = %d",
        // sizeof(tlvType));

        // get tlvLen (32 bits) & remove from queue
        memcpy(&tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
        currentDatap += (sizeof(tlvLen));

        // ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvLen) = %d",
        // sizeof(tlvLen));currentDataprt Thread : tlvType = %d, tlvLen = %d",
        // (int) tlvType, tlvLen);
        //printf("Processing New TLV: Type=%u | Length=%u bytes\n", tlvType, tlvLen);

    switch (tlvType) {
        case MMWDEMO_OUTPUT_MSG_NULL:
            //printf("Skipping NULL message.\n");
            break;

        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
            //printf("Detected Points Message Found.\n");
            sorterState = READ_OBJ_STRUCT;
            break;
        
        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
            // ROS_INFO("DataUARTHandler Sort Thread : Side info TLV");
            sorterState = READ_SIDE_INFO;
            break;

        case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
            //printf("TLV Type=%u | Length=%u\n", tlvType, tlvLen);
            //printf("Range Profile Message Found.\n");
            sorterState = READ_LOG_MAG_RANGE;
            break;

        case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
            //printf("Noise Profile Message Found.\n");
            sorterState = READ_NOISE;
            break;

        case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
          // ROS_INFO("DataUARTHandler Sort Thread : Azimuth Heat TLV");
          sorterState = READ_AZIMUTH;
          break;

        case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
          // ROS_INFO("DataUARTHandler Sort Thread : R/D Heat TLV");
          sorterState = READ_DOPPLER;
          break;

        case MMWDEMO_OUTPUT_MSG_STATS:
          // ROS_INFO("DataUARTHandler Sort Thread : Stats TLV");
          sorterState = READ_STATS;
          break;



        case MMWDEMO_OUTPUT_MSG_MAX:
          // ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
          sorterState = READ_HEADER;
          break;

        default:
          break;
        }
      }

      break;

    case SWAP_BUFFERS:

      pthread_mutex_lock(&countSync_mutex);
      pthread_mutex_unlock(&currentBufp_mutex);

      countSync++;

      if (countSync == COUNT_SYNC_MAX) {
        pthread_cond_signal(&countSync_max_cv);
      }

      pthread_cond_wait(&sort_go_cv, &countSync_mutex);

      pthread_mutex_unlock(&countSync_mutex);
      pthread_mutex_lock(&currentBufp_mutex);

      currentDatap = 0;
      tlvCount = 0;

      sorterState = READ_HEADER;

      break;

    default:
      break;
    }
  }

  pthread_exit(NULL);
}

void DataUARTHandler::start(void) {

  pthread_t uartThread, sorterThread, swapThread,socketSender, parsedSender;

  int iret1, iret2, iret3, iret4, iret5;

  pthread_mutex_init(&countSync_mutex, NULL);
  pthread_mutex_init(&nextBufp_mutex, NULL);
  pthread_mutex_init(&currentBufp_mutex, NULL);
  pthread_mutex_init(&sendQueueMutex, NULL);
  pthread_mutex_init(&parsedQueueMutex, NULL);
  pthread_cond_init(&countSync_max_cv, NULL);
  pthread_cond_init(&read_go_cv, NULL);
  pthread_cond_init(&sort_go_cv, NULL);
  pthread_cond_init(&sendQueueCV, NULL);
  pthread_cond_init(&parsedQueueCV, NULL);

  countSync = 0;
  stopSenderThread = false;  
  stopParsedThread = false;  
  /* Create independent threads each of which will execute function */
  iret1 =
      pthread_create(&uartThread, NULL, this->readIncomingData_helper, this);
  if (iret1) {
    printf("Error - pthread_create() return code: %d\n", iret1);
    rclcpp::shutdown();
  }

  iret2 =
      pthread_create(&sorterThread, NULL, this->sortIncomingData_helper, this);
  if (iret2) {
    printf("Error - pthread_create() return code: %d\n", iret2);
    rclcpp::shutdown();
  }

  iret3 =
      pthread_create(&swapThread, NULL, this->syncedBufferSwap_helper, this);
  if (iret3) {
    printf("Error - pthread_create() return code: %d\n", iret3);
    rclcpp::shutdown();
  }

if (enableSenderThread) {
iret4 = pthread_create(
    &socketSender,
    NULL,
    [](void *context) -> void * {
        static_cast<DataUARTHandler *>(context)->socketSenderThread();
        return nullptr;
    },
    this);

if (iret4) {
    printf("Error - Failed to create socket sender thread: %d\n", iret4);
}
}

iret5 = pthread_create(
    &parsedSender,
    NULL,
    [](void *context) -> void * {
        static_cast<DataUARTHandler *>(context)->parsedDataSocketThread();
        return nullptr;
    },
    this);

if (iret5) {
    printf("Error - Failed to create socket sender thread: %d\n", iret5);
}
  // rclcpp::spin(shared_from_this());
  while (1)
    continue;

  pthread_join(iret1, NULL);
  printf("DataUARTHandler Read Thread joined\n");
  pthread_join(iret2, NULL);
  printf("DataUARTHandler Sort Thread joined\n");
  pthread_join(iret3, NULL);
  printf("DataUARTHandler Swap Thread joined\n");

  if (enableSenderThread) {
  // Lock sendQueueMutex before modifying shared resources
  if (pthread_mutex_lock(&sendQueueMutex) == 0) {
      stopSenderThread = true;            // Set termination flag for sender thread
      pthread_cond_signal(&sendQueueCV);  // Notify sender thread to wake up and exit gracefully

      pthread_mutex_unlock(&sendQueueMutex); // Unlock sendQueueMutex after modifying shared resou  rce
  } else {
      printf("Error locking sendQueueMutex\n");
  }
  
  printf("Waiting for Socket Sender Thread...\n");
  iret4 = pthread_join(iret4, NULL);
  if (!iret4) {
     printf("Socket Thread joined successfully.\n");
  }
  }

    // Lock parserQueueMutex before modifying shared resources
  if (pthread_mutex_lock(&parsedQueueMutex) == 0) {
      stopParsedThread = true;            // Set termination flag for sender thread
      pthread_cond_signal(&parsedQueueCV);  // Notify sender thread to wake up and exit gracefully

      pthread_mutex_unlock(&parsedQueueMutex); // Unlock parsedQueueMutex after modifying shared resou  rce
  } else {
      printf("Error locking parsedQueueMutex\n");
  }
  
  printf("Waiting for Parsed Socket Sender Thread...\n");
  iret5 = pthread_join(parsedSender, NULL);
  if (!iret5) {
     printf("Socket Parsed Sender Thread joined successfully.\n");
  }

  pthread_mutex_destroy(&countSync_mutex);
  pthread_mutex_destroy(&nextBufp_mutex);
  pthread_mutex_destroy(&currentBufp_mutex);
  if (enableSenderThread) {
  pthread_mutex_destroy(&sendQueueMutex);}
  pthread_mutex_destroy(&parsedQueueMutex);
  pthread_cond_destroy(&countSync_max_cv);
  pthread_cond_destroy(&read_go_cv);
  pthread_cond_destroy(&sort_go_cv);
  if (enableSenderThread) {
  pthread_cond_destroy(&sendQueueCV);}
  pthread_cond_destroy(&parsedQueueCV);

}

void *DataUARTHandler::readIncomingData_helper(void *context) {
  return (static_cast<DataUARTHandler *>(context)->readIncomingData());
}

void *DataUARTHandler::sortIncomingData_helper(void *context) {
  return (static_cast<DataUARTHandler *>(context)->sortIncomingData());
}

void *DataUARTHandler::syncedBufferSwap_helper(void *context) {
  return (static_cast<DataUARTHandler *>(context)->syncedBufferSwap());
}

void DataUARTHandler::visualize(
  const ti_mmwave_ros2_interfaces::msg::RadarScan &msg) {
  // visualization_msgs::msg::Marker marker;
  auto marker = visualization_msgs::msg::Marker();

  marker.header.frame_id = frameID;
  marker.header.stamp = rclcpp::Clock().now();
  marker.id = msg.point_id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.lifetime = rclcpp::Duration(tfr, 0);
  marker.action = marker.ADD;

  marker.pose.position.x = msg.x;
  marker.pose.position.y = msg.y;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;

  marker.scale.x = .03;
  marker.scale.y = .03;
  marker.scale.z = .03;

  marker.color.a = 1;
  marker.color.r = (int)255 * msg.intensity;
  marker.color.g = (int)255 * msg.intensity;
  marker.color.b = 1;

  // marker_pub->publish(marker);
}

// Funtions to handle socket Data Transfers
void DataUARTHandler::setupSocket(const std::string &server_ip, int server_port) {
    // Create a TCP socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        printf("Failed to create socket\n");
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr)); // Zero-initialize structure
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);

    // Convert IP address from string format into binary form
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
        printf("Invalid server address: %s\n", server_ip.c_str());
        close(socket_fd);
        return;
    }

    // Attempt connection
    if (connect(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("Failed to connect to %s:%d\n", server_ip.c_str(), server_port);
        close(socket_fd);
        return;
    }

    printf("Connected successfully to %s:%d\n", server_ip.c_str(), server_port);
    socket_connected = true;
}


void DataUARTHandler::sendRawDataOverSocket(const std::vector<uint8_t> &data) {
    if (!socket_connected || socket_fd < 0) {
        printf("Socket not connected. Unable to send data.\n");
        return;
    }

    ssize_t bytes_sent = send(socket_fd, data.data(), data.size(), 0);

    if (bytes_sent < 0) {
        printf("Failed to send data over socket.\n");
        close(socket_fd);         // Close invalid connection gracefully.
        socket_connected = false; // Mark as disconnected.
        
        /* Optionally retry connecting here */
        
    } else {
        printf("Sent %ld bytes of raw sensor data over socket.\n", bytes_sent);
    }
}

void DataUARTHandler::socketSenderThread() {
    while (!stopSenderThread) {
        // Lock the send queue mutex before checking or modifying shared resources
        if (pthread_mutex_lock(&sendQueueMutex) != 0) {
            printf("Error locking send queue mutex in socketSenderThread\n");
            continue;
        }

        // Wait until there's data in the queue or we're stopping
        while (sendQueue.empty() && !stopSenderThread) {
    pthread_cond_wait(&sendQueueCV, &sendQueueMutex); // Wait until signaled
	}

        if (stopSenderThread) { 
            pthread_mutex_unlock(&sendQueueMutex); 
            break; 
        }

        // Get packet from front of queue
        auto packet = sendQueue.front();
        sendQueue.pop(); // Remove it from the queue

         // Unlock before sending data over TCP socket so other threads can access the queue.
        if (pthread_mutex_unlock(&sendQueueMutex) != 0) {
            printf("Error unlocking send queue mutex in socketSenderThread\n");
            continue;
        }

        // Send packet over TCP socket
        sendRawDataOverSocket(packet);
    }

    printf("Socket sender thread exiting...\n");
}

void DataUARTHandler::sendParsedDataOverSocket(const std::string &data) {
    if (!socket_connected || socket_fd < 0) {
        setupSocket(server_ip, server_port); // Attempt to reconnect
        if (!socket_connected) {
            printf("Unable to establish socket connection. Cannot send parsed data.\n");
            return;
        }
    }

    size_t totalBytesSent = 0; // Track total bytes sent
    size_t remainingBytes = data.size(); // Bytes left to send

    while (remainingBytes > 0) {
        ssize_t bytes_sent = send(socket_fd, data.data() + totalBytesSent, remainingBytes, MSG_NOSIGNAL);

        if (bytes_sent <= 0) {
            perror("Failed to send parsed radar data over socket.");
            close(socket_fd);         // Close invalid connection gracefully
            socket_connected = false; // Mark as disconnected
            
            /* Optionally retry connecting here */
            return;
        }

        totalBytesSent += bytes_sent; // Update total bytes sent
        remainingBytes -= bytes_sent; // Update remaining bytes
    }

    // Optional: Log successful transmission size for debugging
    printf("Successfully sent %zu bytes over socket.\n", totalBytesSent);
}

void DataUARTHandler::parsedDataSocketThread() {
    while (!stopParsedThread) {

      // Lock the send queue mutex before checking or modifying shared resources
        if (pthread_mutex_lock(&parsedQueueMutex) != 0) {
            printf("Error locking send queue mutex in parsedDataSocketThread\n");
            continue;
        }


        // Wait until there's data in the queue or we're stopping
        while (parsedDataQueue.empty() && !stopParsedThread) {
          pthread_cond_wait(&parsedQueueCV, &parsedQueueMutex); // Wait until signaled
	      }

        if (stopParsedThread) { 
            pthread_mutex_unlock(&parsedQueueMutex); 
            break; 
        }

        // Retrieve serialized packet from front of queue
        std::vector<unsigned char> packetVector = parsedDataQueue.front();
        parsedDataQueue.pop();

        // Unlock before sending data over TCP socket so other threads can access the queue.
        if (pthread_mutex_unlock(&parsedQueueMutex) != 0) {
            printf("Error unlocking send queue mutex in parsedDataSocketThread\n");
            continue;
        }

        // Convert vector to string for transmission over TCP socket
        std::string serializedPacket(packetVector.begin(), packetVector.end());

        // Send packet over TCP socket
         try {
          sendParsedDataOverSocket(serializedPacket);
         } catch (const std::exception& e) {
             fprintf(stderr, "Error sending parsed packet: %s\n", e.what());
         }
     }

     printf("Parsed Data Socket Thread Exiting...\n");
}

// Organize data in json format
std::string DataUARTHandler::serializeRadarPacket(const DataUARTHandler::ParsedRadarPacket &packet) {
    json j;

    // Add timestamp with 6 decimal precision
    j["timestamp"] = std::round(packet.timestamp * 1000000.0) / 1000000.0;

    // Add number of objects
    j["numObj"] = packet.numObjects;

    // Initialize arrays for object-level data
    std::vector<double> ranges;
    std::vector<double> azimuths;
    std::vector<double> elevations;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    std::vector<double> velocities;
    std::vector<double> intensities;
    std::vector<double> rangeProfilePrecise;

    // Process each detected object with fixed precision
    for (const auto& obj : packet.objects) {
        ranges.push_back(std::round(obj.range * 1000.0) / 1000.0);
        azimuths.push_back(std::round(obj.azimuth * 1000.0) / 1000.0);
        elevations.push_back(std::round(obj.elevation * 1000.0) / 1000.0);
        xs.push_back(std::round(obj.x * 1000.0) / 1000.0);
        ys.push_back(std::round(obj.y * 1000.0) / 1000.0);
        zs.push_back(std::round(obj.z * 1000.0) / 1000.0);
        velocities.push_back(std::round(obj.velocity * 1000.0) / 1000.0);
        intensities.push_back(std::round(obj.intensity * 1000.0) / 1000.0);
    }

    // Add object-level arrays to JSON
    j["range"] = ranges;
    j["azimuth"] = azimuths;
    j["elevation"] = elevations;
    j["x"] = xs;
    j["y"] = ys;
    j["z"] = zs;
    j["v"] = velocities;
    j["snr"] = intensities;

    // Add range profile to JSON
    // Process range profile with fixed precision
    for (const auto& value : packet.rangeProfile) {
        rangeProfilePrecise.push_back(std::round(value * 1000.0) / 1000.0);
    }
    j["rangeProfile"] = rangeProfilePrecise;

    // Return the formatted JSON string directly
    return j.dump()+ "\n"; 
}

// functions for RangeProfileparsing
int16_t DataUARTHandler::getInt16Q7_9(const uint8_t *data) {
    // Extract LSB and MSB from input bytes
    uint8_t lsb = data[0];
    int8_t msb = static_cast<int8_t>(data[1]); // Cast to signed 8-bit integer for proper handling

    // Combine LSB and MSB into a signed 16-bit integer
    return static_cast<int16_t>(lsb | (msb << 8));
}

float DataUARTHandler::log2Lin(int numVirtAnt) {
    return (1.0f / 512.0f) * (std::pow(2, std::ceil(std::log2(numVirtAnt))) / numVirtAnt);
}

float DataUARTHandler::dspFftScaleCompAllLog(int cfgNumRangeBins) {
    return 20 * std::log10(32.0f / cfgNumRangeBins);
}

float DataUARTHandler::rpToDb(float value, int cfgNumRangeBins) {
    float logToLin = log2Lin(12);
    float toDB = 20 * std::log10(2); // Conversion constant for dB
    float dspFftScaleCompAll = dspFftScaleCompAllLog(cfgNumRangeBins);

    // Apply all transformations and return final result
    return std::round((value * logToLin * toDB + dspFftScaleCompAll) * 1000.0f) / 1000.0f; // Round to three decimals
}

// Function to print ParsedRadarPacket details including all info
void DataUARTHandler::printParsedRadarPacket(const ParsedRadarPacket &packet) {
    std::cout << "=== Parsed Radar Packet ===" << std::endl;
    
    // Print timestamp and number of objects
    std::cout << "Timestamp: " << packet.timestamp << std::endl;
    std::cout << "Number of Objects: " << packet.numObjects << std::endl;

    // Loop through each detected object and print its details
    for (const auto &obj : packet.objects) {
        std::cout << "Object ID: " << obj.point_id << ":" << std::endl;
        std::cout << " - Position: (" 
                  << obj.x << ", "
                  << obj.y << ", "
                  << obj.z << ") m" 
                  << std::endl;
        std::cout << " - Velocity: " 	<< obj.velocity <<" m/s"<<std :: endl ;
        std::cout<< "- Intensity: "<<obj.intensity<<" dB"<<std :: endl ;
        std::cout<< "- Range: "<<obj.range<<" m"<<std :: endl ;
        std::cout<< "- Azimuth: "<<obj.azimuth<<" degrees"<<std :: endl ;
        std::cout<< "- Elevation: "<<obj.elevation<<" degrees"<<std :: endl ;

        if (&obj != &packet.objects.back()) {  // Check if it's not the last object for spacing.
            std::cout<<std :: endl ;  
        }
   }

   // Print range profile data if available
   if (!packet.rangeProfile.empty()) {
       std::cout << "Range Profile:" << std::endl;
       for (size_t i = 0; i < packet.rangeProfile.size(); ++i) {
           std::cout 	<< "["<<i+1<<"] : "<<packet.rangeProfile[i] <<" dB,  " 	 ; 
       }
   } else {
       std::cout 	<< "No Range Profile Data Available." 	<<std :: endl ; 
   }
}

void DataUARTHandler::saveDataToLog(const std::string &serializedData) {
    // Only create directory and generate filename once
    if (logFilePath.empty()) {
        // Create logs directory in current workspace
        std::filesystem::path currentPath = std::filesystem::current_path();
        std::filesystem::path logDir = currentPath / "radar_logs";
        
        if (!std::filesystem::exists(logDir)) {
            try {
                std::filesystem::create_directories(logDir);
                RCLCPP_INFO(this->get_logger(), "Created log directory: %s", logDir.string().c_str());
            } catch (const std::filesystem::filesystem_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
                return;
            }
        }

        // Generate timestamp for filename only once
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << logDir.string() << "/radar_data_" 
                << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S") 
                << ".txt";
        
        logFilePath = filename.str();
    }

    // Open file in append mode using the stored path
    std::ofstream logFile(logFilePath, std::ios::app);
    if (logFile.is_open()) {
        logFile << serializedData << std::endl;
        logFile.close();
    } else {
        fprintf(stderr, "Error: Unable to open log file: %s\n", logFilePath.c_str());
    }
}