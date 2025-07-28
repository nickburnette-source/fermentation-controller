#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SIGNAL_PIN 13 // Define the signal output pin
#define SCAN_TIME 10  // seconds to scan
#define ADDRESS "ec:59:f9:82:6f:2e" // BLE Device Address
#define MIN_CHANGE_INTERVAL 60000 // Minimum interval between pin changes in milliseconds
const float HYST = 5.0; // Hysteresis in Fahrenheit

// Encoder pins
#define CLK_PIN 4
#define DT_PIN 5
#define SW_PIN 14 // Button

// 4-digit 7-segment display pins (common anode)
const int digitPins[4] = {16, 17, 18, 19}; // Digit selectors (d1 left to d4 right)
const int segmentPins[8] = {23, 22, 21, 25, 26, 27, 32, 33}; // a, b, c, d, e, f, g, dp

// 7-segment patterns (bit 0=a, 1=b, 2=c, 3=d, 4=e, 5=f, 6=g)
const uint8_t patterns[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

// Display buffers
char displayDigits[4] = {' ', ' ', ' ', ' '};
bool displayDps[4] = {false, false, false, false};
int currentDigit = 0;

// Encoder state
int lastCLKState;

// Setpoint for testing
float setpoint = 55.0;

// Display toggle
bool displayOn = true;
int lastSWState;
unsigned long lastButtonPress = 0;

// BLE globals
BLEScan* pBLEScan;
unsigned long lastChangeTime = 0; // Last time the signal pin was changed
bool scanning = false;
unsigned long nextScanTime = 0;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getAddress().toString() == ADDRESS) {
      String manufacturerData = advertisedDevice.getManufacturerData();
      if (manufacturerData.length() >= 4) {
        uint8_t byte1 = static_cast<uint8_t>(manufacturerData[1]);
        uint8_t byte2 = static_cast<uint8_t>(manufacturerData[2]);
        int16_t temperatureRaw = (static_cast<int16_t>(byte2) << 8) | byte1;
        float temperature = ((temperatureRaw / 10.0) * 1.8) + 32;
        uint8_t humidity = static_cast<uint8_t>(manufacturerData[3]);
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" F, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        float high_setpoint = setpoint + (HYST / 2.0);
        float low_setpoint = setpoint - (HYST / 2.0);

        unsigned long currentTime = millis();
        if (currentTime - lastChangeTime >= MIN_CHANGE_INTERVAL) {
          if (temperature > high_setpoint && digitalRead(SIGNAL_PIN) == LOW) {
            digitalWrite(SIGNAL_PIN, HIGH);
            lastChangeTime = currentTime;
          } else if (temperature < low_setpoint && digitalRead(SIGNAL_PIN) == HIGH) {
            digitalWrite(SIGNAL_PIN, LOW);
            lastChangeTime = currentTime;
          }
        }
      }
    }
  }
};

void scanComplete(BLEScanResults results) {
  pBLEScan->clearResults();
  scanning = false;
  nextScanTime = millis() + 2000;
}

void updateDisplay() {
  bool is_negative = (setpoint < 0);
  float num = abs(setpoint);
  int whole = (int)num;
  int tenths = (int)round((num - whole) * 10);
  int hundreds = whole / 100;
  int tens = (whole / 10) % 10;
  int ones = whole % 10;

  // Always set tenths and ones
  displayDigits[3] = tenths + '0';
  displayDigits[2] = ones + '0';

  // Decimal point always after the ones place (position 2)
  displayDps[0] = false;
  displayDps[1] = false;
  displayDps[2] = true;
  displayDps[3] = false;

  // Set left digits with blanks for leading positions
  char left0 = ' ';
  char left1 = ' ';
  if (whole >= 10) {
    left1 = tens + '0';
    if (whole >= 100) {
      left0 = hundreds + '0';
    }
  }

  // Handle negative sign by placing '-' in the leftmost position
  if (is_negative) {
    if (whole >= 10) {
      left0 = '-';
    } else {
      left0 = '-';
      left1 = ' ';
    }
  }

  displayDigits[0] = left0;
  displayDigits[1] = left1;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start setup");

  // Encoder pins
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  lastCLKState = digitalRead(CLK_PIN);
  lastSWState = digitalRead(SW_PIN);

  // Setup display pins
  for (int i = 0; i < 4; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], LOW); // Initial off (LOW for common anode)
  }
  for (int i = 0; i < 8; i++) {
    pinMode(segmentPins[i], OUTPUT);
    digitalWrite(segmentPins[i], HIGH); // Initial off (HIGH)
  }

  updateDisplay();

  // BLE setup
  pinMode(SIGNAL_PIN, OUTPUT);
  digitalWrite(SIGNAL_PIN, LOW); // Initialize signal pin to LOW
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);  // active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

  Serial.println("Setup done");
}

void loop() {
  // Read encoder
  int clkState = digitalRead(CLK_PIN);
  if (clkState != lastCLKState && clkState == 1) {
    if (digitalRead(DT_PIN) != clkState) {
      setpoint -= 0.5;
    } else {
      setpoint += 0.5;
    }
    // Updated limits
    if (setpoint < -50.0) setpoint = -50.0;
    if (setpoint > 150.0) setpoint = 150.0;
    lastCLKState = clkState;
    updateDisplay();
  } else {
    lastCLKState = clkState;
  }

  // Read button for toggle
  int swState = digitalRead(SW_PIN);
  if (swState == LOW && lastSWState == HIGH && millis() - lastButtonPress > 50) {
    displayOn = !displayOn;
    lastButtonPress = millis();
  }
  lastSWState = swState;

  // Trigger BLE scan if ready
  if (!scanning && millis() >= nextScanTime) {
    scanning = true;
    pBLEScan->start(SCAN_TIME, scanComplete, false);
  }

  // Multiplex display
  // Turn off all digits
  for (int i = 0; i < 4; i++) {
    digitalWrite(digitPins[i], LOW); // Off (LOW)
  }

  // Set segments for current digit
  char ch = displayDigits[currentDigit];
  bool dp = displayDps[currentDigit];

  if (displayOn) {
    if (ch >= '0' && ch <= '9') {
      int dig = ch - '0';
      uint8_t pat = patterns[dig];
      for (int s = 0; s < 7; s++) {
        digitalWrite(segmentPins[s], (pat & (1 << s)) ? LOW : HIGH); // On: LOW, Off: HIGH
      }
    } else if (ch == '-') {
      for (int s = 0; s < 7; s++) {
        digitalWrite(segmentPins[s], (s == 6) ? LOW : HIGH); // g on for '-'
      }
    } else {
      for (int s = 0; s < 7; s++) {
        digitalWrite(segmentPins[s], HIGH); // Off
      }
    }
    digitalWrite(segmentPins[7], dp ? LOW : HIGH); // DP on: LOW, off: HIGH

    // Turn on current digit
    digitalWrite(digitPins[currentDigit], HIGH); // On (HIGH)
  } else {
    // Display off: set segments off (optional, since digits off)
    for (int s = 0; s < 8; s++) {
      digitalWrite(segmentPins[s], HIGH); // Off
    }
    // Digits already off
  }

  // Persist the digit for a short time
  delay(1);

  // Cycle to next digit
  currentDigit = (currentDigit + 1) % 4;

  // Debug running
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 1000) {
    lastDebug = millis();
    Serial.println("Running");
  }
}