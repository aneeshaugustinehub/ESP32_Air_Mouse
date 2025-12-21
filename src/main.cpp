#include <Arduino.h>
#include <BleMouse.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Initialize the Bluetooth Mouse
BleMouse bleMouse("ESP32 Air Mouse", "Espressif", 100);
Adafruit_MPU6050 mpu;

// Variables for movement
bool sensorConnected = false;

// --- YOUR NEW PIN DEFINITIONS ---
#define SDA_PIN 22
#define SCL_PIN 21

void setup() {
  Wire.setClock(10000); 

  Serial.println("Starting Air Mouse (Slow Mode)...");
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Ensure we use the correct pins

  Serial.println("Starting Air Mouse...");

  // Try to find the sensor at 0x68
  if (!mpu.begin(0x68)) {
    Serial.println("❌ Sensor disconnected! Wiggle the wires.");
    sensorConnected = false;
  } else {
    Serial.println("✅ MPU6050 Ready at 0x68!");
    sensorConnected = true;
    
    // Settings for smooth mouse movement
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Start Bluetooth
  bleMouse.begin();
}

void loop() {
  // 1. SAFETY CHECK: If sensor disconnects, try to reconnect
  if (!sensorConnected) {
    if (mpu.begin(0x68)) {
      Serial.println("✅ Sensor Reconnected!");
      sensorConnected = true;
      // Re-apply settings
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    } else {
      // Print a dot every 500ms so you know it's trying
      Serial.print("."); 
      delay(500);
      return; 
    }
  }

  // 2. READ DATA & MOVE MOUSE
  if (bleMouse.isConnected() && sensorConnected) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // --- SENSITIVITY ---
    // Change '15' to '20' for faster speed, '10' for slower
    int moveX = -(int)(g.gyro.z * 15); 
    int moveY = -(int)(g.gyro.y * 15);
    
    // --- DEADZONE ---
    // Ignore tiny movements (jitter)
    if (abs(moveX) > 1 || abs(moveY) > 1) {
      bleMouse.move(moveX, moveY);
    }
    
    delay(10); // Smoothness delay
  }
}