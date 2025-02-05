#include <esp_now.h>
#include <WiFi.h>

// Define the MAC address of the other ESP32
uint8_t otherEsp32MacAddress[] = {0x3C, 0x71, 0xBF, 0xCE, 0xFF, 0xB0}; // Replace with the actual MAC address

// Define variables to store incoming and outgoing messages
char outgoingMessage[2]; // "0" or "1"

// Function prototypes
void OnDataReceived(const esp_now_recv_info* info, const uint8_t* data, int len);
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);

// Setup function runs once at startup
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);  // Set baud rate
  while (!Serial) {
    ; // Wait for the serial port to be available
  }

  // Initialize Wi-Fi in Station mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW initialization failed. Retrying...");
  delay(1000);  // Wait before retrying
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed again. Restarting...");
    ESP.restart();  // Restart the ESP32 to recover
  }
}


  // Register callback functions for sending and receiving data
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataReceived);

  // Add peer (the other ESP32 device)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, otherEsp32MacAddress, 6);
  peerInfo.channel = 0; // Use the current channel
  peerInfo.encrypt = false; // Disable encryption

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  Serial.println("Failed to add peer. Retrying...");
  delay(1000);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer again. Restarting...");
    ESP.restart();  // Restart to recover from failure
  }
}

}

// Loop function runs repeatedly
void loop() {
  // Check if data is available on the Serial port
  if (Serial.available() > 0) {
    // Read the incoming data until newline
    String data = Serial.readStringUntil('\n');
    
    // Convert the received string to a float value
    float receivedValue = data.toFloat();

    // Prepare the outgoing message based on the received value
    outgoingMessage[0] = (receivedValue > 0.6) ? '1' : '0'; // "1" if value > 0.6, else "0"
    outgoingMessage[1] = '\0'; // Null-terminate the string

    // Send the message over ESP-NOW
    esp_err_t result = esp_now_send(otherEsp32MacAddress, (uint8_t *)outgoingMessage, sizeof(outgoingMessage));
if (result != ESP_OK) {
  Serial.println("Error sending data. Retrying...");
  int retries = 0;
  while (retries < 3 && result != ESP_OK) {
    delay(100);  // Wait before retrying
    result = esp_now_send(otherEsp32MacAddress, (uint8_t *)outgoingMessage, sizeof(outgoingMessage));
    retries++;
  }
  if (result != ESP_OK) {
    Serial.println("Failed to send data after retries. Check connection.");
  }
}

  }
}

// Callback function for data received
void OnDataReceived(const esp_now_recv_info* info, const uint8_t* data, int len) {
  Serial.write(data, len); // Write the received data to Serial Monitor
}

// Callback function for data sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Uncomment for send status debugging
  // Serial.print("Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
