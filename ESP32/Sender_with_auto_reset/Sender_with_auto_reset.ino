#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>

// Define the MAC address of the other ESP32
uint8_t otherEsp32MacAddress[] = {0x3C, 0x71, 0xBF, 0xCE, 0xFF, 0xB0}; // Replace with actual MAC address

//changes Irfan
String incomingData = "";  // Variable to store incoming serial data
unsigned long lastDataTime = 0;  // Timestamp for last received data
const unsigned long timeoutPeriod = 5000;  // 5 seconds timeout period


// Initialize default UART0 (TX = GPIO1, RX = GPIO3)
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);  // Set baud rate as needed
  while (!Serial) {
    ; // Wait for the serial port to be available
  }

  // Initialize Wi-Fi in STA mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  // Register the send callback function
  esp_now_register_send_cb(OnDataSent);

  // Register the receive callback function
  esp_now_register_recv_cb(OnDataReceived);

  // Add peer (the other ESP32 device)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, otherEsp32MacAddress, 6);
  peerInfo.channel = 0; // Use the current channel
  peerInfo.encrypt = false; // Encryption off
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Read the data from UART
    
    //int data = Serial.read();
    //incomingData = Serial.readStringUntil('\n');
    incomingData = Serial.readString();
    lastDataTime = millis();  // Update the timestamp

    // Check if the received message is the termination signal
    if (incomingData == "TERMINATE") {
      Serial.println("Termination signal received. Stopping communication.");
      // Stop transmission or enter a safe state
      while (true) {
        // Stay in an idle state or sleep
        delay(1000);
      }
    }
    
    const char* dataBytes = incomingData.c_str(); 
    size_t dataLength = strlen(dataBytes);
    esp_err_t result = esp_now_send(otherEsp32MacAddress, (uint8_t *)dataBytes, dataLength);
    
    //Serial.write(data);
    // Send the data over ESP-NOW
    //esp_err_t result = esp_now_send(otherEsp32MacAddress, (uint8_t *)&data, sizeof(data));
    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Error sending data");
    }
    
  }

    // Check for data timeout (no data received for 5 seconds)
  if (millis() - lastDataTime > timeoutPeriod) {
    Serial.println("No data received for 5 seconds. Entering idle state.");
    // Take appropriate action, such as resetting the ESP32 or entering low-power mode
    ESP.restart();  // Uncomment this line if you want to reset the ESP32
    delay(1000);  // Stay idle, waiting for data
  }
}

// Callback function for data received
void OnDataReceived(const esp_now_recv_info* info, const uint8_t* data, int len) {
  //Serial.println("Data Received:");
  Serial.write(data, len);
}


// Callback function for data sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("Send Status: ");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
