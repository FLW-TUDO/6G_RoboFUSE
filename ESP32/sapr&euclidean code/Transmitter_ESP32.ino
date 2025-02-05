#include <esp_now.h>
#include <WiFi.h>

// Structure to hold the data from python3
typedef struct struct_message_hold {
    char message[100];
} struct_message_hold;

struct_message_hold myData;

//Structure to receive the message from other ESP_32
typedef struct struct_message_recv{
  char message[100];
} struct_message_recv;

struct_message_recv recievedData;

// MAC Address of the receiver ESP32
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0xCE, 0xFF, 0xB0};

unsigned long lastDataTime = 0;  // Timestamp for last received data
const unsigned long timeoutPeriod = 5000;  // 5 seconds timeout period

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function for data received
void OnDataReceived(const esp_now_recv_info *info, const uint8_t *recievedData, int len) {
  char message[len + 1];
  memcpy(message, recievedData, len);
  message[len] = '\0';
  
  Serial.println(message); // For a new line
}

void setup() {
    // Initialize the Serial Monitor
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for the serial port to be available
    }
    Serial.println("Serial Monitor Ready. Start sending data...");

    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // We don't want to connect to any network

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send callback
    esp_now_register_send_cb(OnDataSent);

    // Register the receive callback function
    esp_now_register_recv_cb(OnDataReceived);

    // Add peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    char incomingData[100];  // Buffer to store incoming data

    if (Serial.available() > 0) {
        int bytesRead = Serial.readBytesUntil('\n', incomingData, sizeof(incomingData) - 1);
        incomingData[bytesRead] = '\0';  // Null-terminate the string
        //Serial.print("Received: ");
        Serial.println(incomingData);
        lastDataTime = millis();

        // Copy the incoming data to the ESP-NOW message structure
        strncpy(myData.message, incomingData, sizeof(myData.message));

        // Send the data via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        
        if (result == ESP_OK) {
            Serial.println("Sent with success");
        } else {
            Serial.println("Error sending the data");
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
