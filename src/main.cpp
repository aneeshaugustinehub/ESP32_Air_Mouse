#include <Arduino.h>
#include <BleMouse.h>
#include <Wire.h>

BleMouse bleMouse("ESP32 Air Mouse", "Espressif", 100);

// Standard Pins
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Wire
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // Standard speed

  Serial.println("Starting Raw Mode...");

  // 2. WAKE UP MPU6050 (It sleeps by default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("❌ Connection Failed. Error Code: ");
    Serial.println(error);
    Serial.println("CHECK IF PIN 21 AND 22 ARE TOUCHING!");
    while(1);
  }
  
  Serial.println("✅ MPU6050 Woke up! Mouse Active.");
  bleMouse.begin();
}

void loop() {
  if (bleMouse.isConnected()) {
    // 3. Request Data (Gyro only)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Starting register for Gyro Data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, 1); // Request 6 bytes

    // 4. Read Data
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // 5. Move Mouse
    // Adjust 150 to change sensitivity
    int moveX = -(GyroZ / 150); 
    int moveY = -(GyroY / 150);

    if (abs(moveX) > 1 || abs(moveY) > 1) {
      bleMouse.move(moveX, moveY);
    }
    
    // Left Click (Boot Button)
    if (digitalRead(0) == LOW) {
      if (!bleMouse.isPressed(MOUSE_LEFT)) bleMouse.press(MOUSE_LEFT);
    } else {
      if (bleMouse.isPressed(MOUSE_LEFT)) bleMouse.release(MOUSE_LEFT);
    }

    delay(10);
  }
}