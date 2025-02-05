#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "esp_task_wdt.h"

// Timeout for receiving ESP-NOW data (10 seconds)
unsigned long lastRecvTime = 0;
const unsigned long timeout = 10000;  // 10 seconds

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0xCF, 0x00, 0xF0};

// Define variables to store incoming and outgoing messages
String outgoingMessage = "";
String incomingMessage = "";

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // No need to print send status
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  char incomingBuffer[100];
  memcpy(incomingBuffer, incomingData, len);
  incomingBuffer[len] = '\0'; // Null-terminate the string
  incomingMessage = String(incomingBuffer);
  
  // Create JSON object with a memory pool of 200 bytes
  StaticJsonDocument<200> doc;
  doc["message"] = incomingMessage;

  // Convert JSON object to string
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Print JSON string to Serial Monitor
  Serial.println(jsonString);
  
  // Clear/reset the buffer/memory
  doc.clear();

  // Update the time when data was last received
  lastRecvTime = millis();

  // Reset the watchdog timer upon receiving data
  esp_task_wdt_reset();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for the serial port to be available
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Define watchdog timer configuration
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 10000,     // Set timeout to 10 seconds
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Monitor all cores
    .trigger_panic = true,   // Trigger panic on timeout
  };

  // Initialize the Watchdog
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);  // Add the current task (loop) to WDT monitoring

  // Initialize the last received time
  lastRecvTime = millis();
}

void loop() {
  // Check if data was received via Serial
  if (Serial.available() > 0) {
    String commandfromJetson = Serial.readString();  // Read the incoming data
    Serial.println(commandfromJetson);  // Print the message

    // Convert the received string to a float value
    float receivedValue = commandfromJetson.toFloat();

    // Prepare the message based on the received value
    outgoingMessage = (receivedValue > 0.4) ? "1" : "0";

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) outgoingMessage.c_str(), outgoingMessage.length() + 1);
  }

  // Check if no data has been received within the timeout period
  if (millis() - lastRecvTime > timeout) {
    Serial.println("No data received in 10 seconds, resetting...");
    esp_task_wdt_reset();  // Manually reset the watchdog to trigger a reset
  }

  delay(100);  // Delay to prevent flooding
}
