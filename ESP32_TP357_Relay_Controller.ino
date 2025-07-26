#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SCAN_TIME 10  // seconds to scan
#define ADDRESS "ec:59:f9:82:6f:2e" // BLE Device Address

#define SIGNAL_PIN 13 // Define the signal output pin
#define HIGH_SETPOINT 57.5 // High temperature setpoint in Fahrenheit
#define LOW_SETPOINT 52.5 // Low temperature setpoint in Fahrenheit
#define MIN_CHANGE_INTERVAL 60000 // Minimum interval between pin changes in milliseconds

BLEScan* pBLEScan;
unsigned long lastChangeTime = 0; // Last time the signal pin was changed

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getAddress().toString() == ADDRESS) {
      String manufacturerData = advertisedDevice.getManufacturerData();
      if (manufacturerData.length() >= 4) {
        uint16_t temperatureRaw = (uint16_t)(manufacturerData[1] | manufacturerData[2]);
        float temperature = ((temperatureRaw / 10.0) * 1.8) + 32;
        uint8_t humidity = static_cast<uint8_t>(manufacturerData[3]);
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" F, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        unsigned long currentTime = millis();
        if (currentTime - lastChangeTime >= MIN_CHANGE_INTERVAL) {
          if (temperature > HIGH_SETPOINT && digitalRead(SIGNAL_PIN) == LOW) {
            digitalWrite(SIGNAL_PIN, HIGH);
            lastChangeTime = currentTime;
          } else if (temperature < LOW_SETPOINT && digitalRead(SIGNAL_PIN) == HIGH) {
            digitalWrite(SIGNAL_PIN, LOW);
            lastChangeTime = currentTime;
          }
        }
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW); // Initialize signal pin to LOW
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);  // active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
  pBLEScan->start(SCAN_TIME, false);
  pBLEScan->clearResults();   // delete results from BLEScan buffer to release memory
  delay(2000);
}