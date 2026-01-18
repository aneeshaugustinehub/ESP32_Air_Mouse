Bluetooth Low Energy (BLE) Air Mouse using an ESP32 microcontroller and motion sensors.

This project turns your ESP32 into a wireless air mouse that moves the cursor on a host computer or mobile device by detecting 3-axis motion from an inertial sensor (e.g., MPU6050) and sending HID mouse events over Bluetooth.
Features

    BLE mouse functionality (pointer movement, clicks)

    Motion/tilt-based cursor control using an IMU (e.g., MPU6050)

    Left/right click support

    Low-power and compact design (ESP32 dev board + sensor)

    Compatible with Android, Windows, Linux and many BLE HID hosts

Hardware Requirements
Component	Description
ESP32 Dev Board	Any ESP32 with BLE support
MPU6050 IMU	6-axis accelerometer + gyroscope
Tactile Buttons	For left and right clicks (optional)
Power Source	USB cable or Li-ion battery
Wires/Breadboard	For prototyping
Wiring

Connect the MPU6050 to the ESP32 I2C pins:

MPU6050 VCC → ESP32 3.3V
MPU6050 GND → ESP32 GND
MPU6050 SDA → ESP32 GPIO21 (default)
MPU6050 SCL → ESP32 GPIO22 (default)
Button Left → Any GPIO + pull-down
Button Right → Any GPIO + pull-down

Adjust pins in your firmware as needed.
Software Dependencies

    ESP-IDF or Arduino ESP32 development environment

        ESP-IDF v5.x (if using native IDF) or Arduino core for ESP32

    IMU library (e.g., MPU6050 or I2Cdevlib)

    BLE Mouse library (e.g., ESP32-BLE-Mouse) for HID over BLE

Installation
ESP-IDF

    Install ESP-IDF following the official documentation.

    Clone this repository:

    git clone https://github.com/aneeshaugustinehub/ESP32_Air_Mouse.git

    Enter project directory and configure:

    idf.py menuconfig

    Build and flash:

    idf.py build flash monitor

Arduino IDE

    Install ESP32 board support in Arduino IDE.

    Add required libraries:

        MPU6050 / I2Cdevlib

        ESP32-BLE-Mouse

    Open the project .ino file (if provided).

    Upload to your ESP32.

Usage

After flashing:

    Enable Bluetooth on your host device.

    Pair with the ESP32 Air Mouse (it should appear as a BLE mouse).

    Move the ESP32 unit to control the cursor.

    Press configured buttons for left/right clicks.

Calibration

Depending on your sensor and mounting orientation:

    Adjust motion scaling

    Tune the IMU filter parameters

    Modify dead zones

Calibration code blocks are available in src/ (modify as needed).
Example Code Snippet

Below is a simplified example of using a BLE mouse library:

#include <BleMouse.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
BleMouse bleMouse;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    bleMouse.begin();
}

void loop() {
    if (bleMouse.isConnected()) {
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);
        bleMouse.move(ax / 100, ay / 100);
    }
    delay(10);
}

This snippet illustrates reading sensor data and sending relative movement.
Project Structure

/include     – Header files
/src         – Source files implementing motion & BLE
/test        – Test code (optional)
platformio.ini – Build configuration (PlatformIO)

Contributing

Contributions are welcome:

    Fork the repository.

    Create your feature branch.

    Open a pull request with a clear description of changes.

License

Specify the license used (e.g., MIT, GPL-3.0) here.
