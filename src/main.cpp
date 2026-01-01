#include <Arduino.h>
#include <BleMouse.h>
#include <Wire.h>

// --- KALMAN FILTER CLASS ---
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
// --------------------------

BleMouse bleMouse("Air Mouse", "Espressif", 100);

// --- PINS (4 & 15) ---
#define SDA_PIN 4 
#define SCL_PIN 15

#define MPU_ADDR 0x68
#define LED_PIN 2     

// --- FILTER SETTINGS ---
// (Measurement Noise, Estimation Noise, Process Noise)
// Process Noise (0.01): Lower = Smoother, Higher = More Responsive
SimpleKalmanFilter kfX(2, 2, 0.01);
SimpleKalmanFilter kfY(2, 2, 0.01);

float calibX = 0, calibY = 0, calibZ = 0;
int deadzone = 3;     

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); 

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  
  Wire.write(0);     
  Wire.endTransmission();
  
  Serial.println("✅ Sensor Active.");

  Serial.println("--------------------------------");
  Serial.println("   DO NOT MOVE THE MOUSE!       ");
  Serial.println("   Calibrating for 3 seconds... ");
  Serial.println("--------------------------------");
  
  long sumX = 0, sumY = 0, sumZ = 0;
  int numReadings = 1000;

  digitalWrite(LED_PIN, HIGH);
  delay(10);
  
  for (int i = 0; i < numReadings; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, 1);

    int16_t x = Wire.read() << 8 | Wire.read();
    int16_t y = Wire.read() << 8 | Wire.read();
    int16_t z = Wire.read() << 8 | Wire.read();

    sumX += x;
    sumY += y;
    sumZ += z;
    delay(3);
  }

  digitalWrite(LED_PIN, LOW); 

  calibX = sumX / numReadings;
  calibY = sumY / numReadings;
  calibZ = sumZ / numReadings;

  Serial.println("Calibration Done!");
  bleMouse.begin();
}

void loop() {
  if (bleMouse.isConnected()) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, 1);

    int16_t rawX = Wire.read() << 8 | Wire.read();
    int16_t rawY = Wire.read() << 8 | Wire.read();
    int16_t rawZ = Wire.read() << 8 | Wire.read();

    float gyroX = rawX - calibX;
    float gyroY = rawY - calibY;
    float gyroZ = rawZ - calibZ;

    // --- KALMAN FILTERING ---
    // We pass the noisy raw data into the filter
    // The filter returns the smooth "Estimated" value
    
    // X Axis uses Gyro Z (Left/Right)
    float smoothX = kfX.updateEstimate(gyroZ);
    
    // Y Axis uses Gyro Y (Up/Down)
    float smoothY = kfY.updateEstimate(gyroX);

    // --- MOVEMENT ---
    // Note: We use the "smooth" variables now, not the raw gyro
    int moveX = (smoothX / 450); 
    int moveY = (smoothY / 500);

    if (abs(moveX) < deadzone) moveX = 0;
    if (abs(moveY) < deadzone) moveY = 0;

    if (moveX != 0 || moveY != 0) {
      bleMouse.move(moveX, moveY);
    }
    
    if (digitalRead(0) == LOW) {
      if (!bleMouse.isPressed(MOUSE_LEFT)) bleMouse.press(MOUSE_LEFT);
    } else {
      if (bleMouse.isPressed(MOUSE_LEFT)) bleMouse.release(MOUSE_LEFT);
    }

    delay(10); 
  }
}