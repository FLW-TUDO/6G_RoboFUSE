#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_
#define PACKET_SIZE_THRESHOLD 1024 
#include "ti_mmwave_ros2_interfaces/msg/radar_scan.hpp"
#include "ti_mmwave_ros2_pkg/visibility_control.h"
// #include "more_interfaces/msg/address_book.hpp"
#include "ti_mmwave_ros2_pkg/mmWave.hpp"
#include <ti_mmwave_ros2_pkg/json.hpp>
//using json = nlohmann::json;
using json = nlohmann::ordered_json;
// #include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <queue>
#include <algorithm>
#include <cmath>


// #include "ros/ros.h"
// #include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <visualization_msgs/msg/marker.hpp>
#define COUNT_SYNC_MAX 2

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using RadarScan = ti_mmwave_ros2_interfaces::msg::RadarScan;
using Marker = visualization_msgs::msg::Marker;

class DataUARTHandler : public rclcpp::Node {

public:
  /*Constructor*/
  // void DataUARTHandler(ros::NodeHandle* nh) :
  // currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {}
  // DataUARTHandler(ros::NodeHandle *nh);
  COMPOSITION_PUBLIC
  DataUARTHandler();

  COMPOSITION_PUBLIC
  void setPublishers(
      const rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub_in
      //const rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub_in,
      //const rclcpp::Publisher<Marker>::SharedPtr marker_pub_in
      );

  void onInit();

  void setNamespace(const std::string &ns);

  void setFrameID(char *myFrameID);

  /*User callable function to set the UARTPort*/
  void setUARTPort(char *mySerialPort);

  /*User callable function to set the BaudRate*/
  void setBaudRate(int myBaudRate);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);

  /*User callable function to set maxAllowedElevationAngleDeg*/
  void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

  int16_t getInt16Q7_9(const uint8_t *data);
  float log2Lin(int numVirtAnt);
  float dspFftScaleCompAllLog(int cfgNumRangeBins);
  float rpToDb(float value, int cfgNumRangeBins);

  /*User callable function to start the handler's internal threads*/
  void start(void);

  /*Helper functions to allow pthread compatability*/
  static void *readIncomingData_helper(void *context);

  static void *sortIncomingData_helper(void *context);

  static void *syncedBufferSwap_helper(void *context);

  void callbackGlobalParam(
      std::shared_future<std::vector<rclcpp::Parameter>> future);
      
  // Configure the TCP socket (e.g., set IP address and port)
  void setupSocket(const std::string &server_ip, int server_port);
  

  void sendRawDataOverSocket(const std::vector<uint8_t> &data);
  
  void socketSenderThread();


  void sendParsedDataOverSocket(const std::string &data);
  
  void parsedDataSocketThread();


  struct ParsedRadarObject {
      uint16_t point_id;   // Unique ID for each detected object
      float x;             // X-coordinate in meters
      float y;             // Y-coordinate in meters
      float z;             // Z-coordinate in meters
      float velocity;      // Velocity of the object in m/s
      float intensity;     // SNR or peak value (dB)
      float range;         // Range from radar sensor (meters)
      float azimuth;       // Azimuth angle (degrees)
      float elevation;     // Elevation angle (degrees)
  };

  struct ParsedRadarPacket {
      double timestamp;                       // Timestamp of the radar frame
      uint32_t numObjects;                   // Number of detected objects
      std::vector<ParsedRadarObject> objects;// List of detected objects

      std::vector<float> rangeProfile;       // Range profile values for all bins (in dB)
  };
  void printParsedRadarPacket(const ParsedRadarPacket &packet); 
 std::string serializeRadarPacket(const ParsedRadarPacket &packet);

  

  /*Sorted mmwDemo Data structure*/
  mmwDataPacket mmwData;
  

private:
  int nr;
  int nd;
  int ntx;
  float fs;
  float fc;
  float BW;
  float PRI;
  float tfr;
  float max_range;
  float vrange;
  float max_vel;
  float vvel;
	
  bool enableSenderThread = false;

  std::string server_ip = "127.0.0.1"; // Default to localhost
  int server_port = 65432;             // Default port number
  int socket_fd;                       // Socket file descriptor
  struct sockaddr_in server_addr;      // Server address structure
  bool socket_connected = false;       // Flag indicating if the socket is connected
  
       // Thread for sending data over TCP
  std::queue<std::vector<uint8_t>> sendQueue; // Queue for storing packets
  pthread_mutex_t sendQueueMutex;       // Mutex for socket queue
  pthread_cond_t sendQueueCV;  	// Condition variable for socket sender thread
  bool stopSenderThread = false;              // Flag to stop sender thread
  
  // Shared resources for parsed radar data
  std::queue<std::vector<uint8_t>> parsedDataQueue; // Queue for storing serialized parsed packets
  pthread_mutex_t parsedQueueMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t parsedQueueCV = PTHREAD_COND_INITIALIZER;
  // Flag to stop the thread safely
  bool stopParsedThread = false;

  std::string logFilePath;

  char *frameID;
  /*Contains the name of the serial port*/
  char *dataSerialPort;

  /*Contains the baud Rate*/
  int dataBaudRate;

  /*Contains the max_allowed_elevation_angle_deg (points with elevation angles
    outside +/- max_allowed_elevation_angle_deg will be removed)*/
  int maxAllowedElevationAngleDeg;

  /*Contains the max_allowed_azimuth_angle_deg (points with azimuth angles
    outside +/- max_allowed_azimuth_angle_deg will be removed)*/
  int maxAllowedAzimuthAngleDeg;

  /*Mutex protected variable which synchronizes threads*/
  int countSync;

  /*Read/Write Buffers*/
  std::vector<uint8_t> pingPongBuffers[2];

  /*Pointer to current data (sort)*/
  std::vector<uint8_t> *currentBufp;

  /*Pointer to new data (read)*/
  std::vector<uint8_t> *nextBufp;

  /*Mutex protecting the countSync variable */
  pthread_mutex_t countSync_mutex;

  /*Mutex protecting the nextBufp pointer*/
  pthread_mutex_t nextBufp_mutex;

  /*Mutex protecting the currentBufp pointer*/
  pthread_mutex_t currentBufp_mutex;

  /*Condition variable which blocks the Swap Thread until signaled*/
  pthread_cond_t countSync_max_cv;

  /*Condition variable which blocks the Read Thread until signaled*/
  pthread_cond_t read_go_cv;

  /*Condition variable which blocks the Sort Thread until signaled*/
  pthread_cond_t sort_go_cv;

  /*Swap Buffer Pointers Thread*/
  void *syncedBufferSwap(void);

  /*Checks if the magic word was found*/
  int isMagicWord(uint8_t last8Bytes[8]);

  /*Read incoming UART Data Thread*/
  void *readIncomingData(void);

  /*Sort incoming UART Data Thread*/
  void *sortIncomingData(void);

  void visualize(const RadarScan &msg);

  // Add to private or public section as needed:
void saveDataToLog(const std::string &serializedData);

  rclcpp::Publisher<PointCloud2>::SharedPtr DataUARTHandler_pub;
  rclcpp::Publisher<RadarScan>::SharedPtr radar_scan_pub;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub;
  
  std::string ns;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
};

#endif
