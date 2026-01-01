#include <Arduino.h>
#include <BleMouse.h>
#include <Wire.h>

// --- KALMAN FILTER CLASS (Compacted for speed) ---
class SimpleKalmanFilter {
public:
  SimpleKalmanFilter(float mea_e, float est_e, float q);
  float updateEstimate(float mea);
private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate = 0;
  float _last_estimate = 0;
  float _kalman_gain = 0;
};

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q) {
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
}

float SimpleKalmanFilter::updateEstimate(float mea) {
  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

// --- SETUP ---
BleMouse bleMouse("Air Mouse", "Espressif", 100);

// Pins
#define SDA_PIN 4 
#define SCL_PIN 15
#define MPU_ADDR 0x68
#define LED_PIN 2     

// Kalman Filters
SimpleKalmanFilter kfX(2, 2, 0.01);
SimpleKalmanFilter kfY(2, 2, 0.01);

// Variables
float calibX = 0, calibY = 0, calibZ = 0;
int deadzone = 1; 
unsigned long lastTime = 0;  // For the timer optimization

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // OPTIMIZATION 1: Increase I2C Speed to 400kHz (Fast Mode)
  Wire.setClock(400000); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission();
  
  Serial.println("✅ Sensor Active.");
  Serial.println("   DO NOT MOVE THE MOUSE! Calibrating...");

  digitalWrite(LED_PIN, HIGH);
  delay(10); // Short wait
  
  long sumX = 0, sumY = 0, sumZ = 0;
  int numReadings = 1000;
  
  for (int i = 0; i < numReadings; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, 1);

    // Efficient bit-shifting read
    sumX += (int16_t)(Wire.read() << 8 | Wire.read());
    sumY += (int16_t)(Wire.read() << 8 | Wire.read());
    sumZ += (int16_t)(Wire.read() << 8 | Wire.read());
    delay(2); // Reduced calibration delay slightly
  }

  digitalWrite(LED_PIN, LOW); 

  calibX = sumX / numReadings;
  calibY = sumY / numReadings;
  calibZ = sumZ / numReadings;

  Serial.println("Calibration Done!");
  bleMouse.begin();
}

void loop() {
  // OPTIMIZATION 2: Non-blocking Timer (Replaces delay)
  // This keeps the Bluetooth connection healthier
  if (millis() - lastTime > 10) {
    lastTime = millis();
    
    if (bleMouse.isConnected()) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, 1);

      int16_t rawX = Wire.read() << 8 | Wire.read();
      int16_t rawY = Wire.read() << 8 | Wire.read();
      int16_t rawZ = Wire.read() << 8 | Wire.read();

      // OPTIMIZATION 3: Direct Math (Skip intermediate variables)
      float smoothX = kfX.updateEstimate(rawZ - calibZ); // Gyro Z
      float smoothY = kfY.updateEstimate(rawX - calibX); // Gyro X

      int moveX = (smoothX / 200); 
      int moveY = (smoothY / 250);

      // Simple deadzone check
      if (abs(moveX) < 1) moveX = 0;
      if (abs(moveY) < 1) moveY = 0;

      if (moveX != 0 || moveY != 0) {
        bleMouse.move(moveX, moveY);
      }
      
      // Click Logic
      if (digitalRead(0) == LOW) {
        if (!bleMouse.isPressed(MOUSE_LEFT)) bleMouse.press(MOUSE_LEFT);
      } else {
        if (bleMouse.isPressed(MOUSE_LEFT)) bleMouse.release(MOUSE_LEFT);
      }
    }
  }
}