// HiPNUC IMU Data Decoder - JSON Output Version
#include "hipnuc_dec.h"

const int LED_PIN = 13;
hipnuc_raw_t hipnuc_raw;

// Frame rate calculation
unsigned long lastSecond = 0;
unsigned long frameCount = 0;
float currentFPS = 0;

// Display control
unsigned long lastDisplay = 0;
const int DISPLAY_RATE = 200; // 5 times per second

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    
    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw_t));
    
    // Print basic system info
    printBasicInfo();
    
    Serial.println("HiPNUC Decoder Ready");
    
    // 3 second countdown
    startupCountdown();
    
    Serial.println("Data acquisition started");
    Serial.println("========================");
}

void printBasicInfo() {
    Serial.println("System Information:");
    Serial.print("MCU Frequency: ");
    Serial.print(F_CPU / 1000000);
    Serial.println(" MHz");
    Serial.println("Baud Rate: 115200");
    
    #if defined(ESP32)
        Serial.println("Board: ESP32");
        Serial.print("Free Heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
    #elif defined(ESP8266)
        Serial.println("Board: ESP8266");
        Serial.print("Free Heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
    #elif defined(ARDUINO_AVR_MEGA2560)
        Serial.println("Board: Arduino Mega2560");
    #elif defined(ARDUINO_AVR_UNO)
        Serial.println("Board: Arduino Uno");
    #else
        Serial.println("Board: Unknown");
    #endif
    
    Serial.println("Version: 1.0");
    Serial.println("------------------------");
}

void startupCountdown() {
    Serial.println("Starting in:");
    for (int i = 3; i > 0; i--) {
        Serial.print(i);
        Serial.println("...");
        
        // Blink LED during countdown
        digitalWrite(LED_PIN, HIGH);
        delay(500);
        digitalWrite(LED_PIN, LOW);
        delay(500);
    }
}

void loop() {
    unsigned long now = millis();
    
    // Read and decode data
    while (Serial1.available()) {
        uint8_t data = Serial1.read();
        
        if (hipnuc_input(&hipnuc_raw, data) > 0) {
            frameCount++;
            digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
        }
    }
    
    // Calculate FPS every second
    if (now - lastSecond >= 1000) {
        currentFPS = frameCount;
        frameCount = 0;
        lastSecond = now;
    }
    
    // Display data at 5Hz
    if (now - lastDisplay >= DISPLAY_RATE) {
        displayData();
        lastDisplay = now;
    }
}

void displayData() {
    Serial.print("{");
    Serial.print("\"fps\":");
    Serial.print(currentFPS, 1);
    Serial.print(",\"time\":");
    Serial.print(millis()/1000.0, 1);
    
    // Display IMU data if available
    if (hipnuc_raw.hi91.tag == 0x91) {
        hi91_t* imu = &hipnuc_raw.hi91;
        
        Serial.print(",\"imu\":{");
        Serial.print("\"acc\":[");
        Serial.print(imu->acc[0]*9.8, 3); Serial.print(",");
        Serial.print(imu->acc[1]*9.8, 3); Serial.print(",");
        Serial.print(imu->acc[2]*9.8, 3);
        Serial.print("],\"gyr\":[");
        Serial.print(imu->gyr[0], 2); Serial.print(",");
        Serial.print(imu->gyr[1], 2); Serial.print(",");
        Serial.print(imu->gyr[2], 2);
        Serial.print("],\"euler\":{");
        Serial.print("\"roll\":"); Serial.print(imu->roll, 2);
        Serial.print(",\"pitch\":"); Serial.print(imu->pitch, 2);
        Serial.print(",\"yaw\":"); Serial.print(imu->yaw, 2);
        Serial.print("}}");
    }
    
    // Display INS data if available
    else if (hipnuc_raw.hi81.tag == 0x81) {
        hi81_t* ins = &hipnuc_raw.hi81;
        Serial.print(",\"ins\":{");
        Serial.print("\"lat\":"); Serial.print(ins->ins_lat * 1e-7, 6);
        Serial.print(",\"lon\":"); Serial.print(ins->ins_lon * 1e-7, 6);
        Serial.print(",\"sats\":"); Serial.print(ins->nv_pos);
        Serial.print("}");
    }
    
    Serial.println("}");
}
