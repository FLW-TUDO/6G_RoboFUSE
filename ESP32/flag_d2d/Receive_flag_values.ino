#include <esp_now.h>
#include <WiFi.h>

// Structure to hold the received data
typedef struct struct_message {
    char message[100];
} struct_message;

struct_message incomingData;
unsigned long lastDataTime = 0;  // Timestamp for last received data
const unsigned long timeoutPeriod = 5000;  // 5 seconds timeout period

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    char message[len + 1];
    memcpy(message, incomingData, len);
    message[len] = '\0';

    //Serial.print("Received Message: ");
    Serial.println(message);
}


void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // We don't want to connect to any network

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register for the callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
}


void loop() {
    // Nothing to do in loop, data will be received via the callback
    lastDataTime = millis();
    if (millis() - lastDataTime > timeoutPeriod) {
    //Serial.println("No data received for 5 seconds. Entering idle state.");
    // Take appropriate action, such as resetting the ESP32 or entering low-power mode
    ESP.restart();  // Uncomment this line if you want to reset the ESP32
    delay(1000);  // Stay idle, waiting for data
  }
}
