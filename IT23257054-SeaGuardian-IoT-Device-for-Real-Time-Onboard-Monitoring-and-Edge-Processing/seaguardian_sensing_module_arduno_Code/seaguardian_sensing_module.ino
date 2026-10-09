#include <WiFi.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

// =====================================================
// Wi-Fi Details
// =====================================================

const char* ssid = "Chamika";
const char* password = "1234567890";


// =====================================================
// PIN DEFINITIONS
// =====================================================

// Existing LEDs
const int POWER_LED = 4;      // Yellow - System ON
const int WIFI_LED  = 2;      // Blue - Wi-Fi status

// System status LEDs
const int NORMAL_LED  = 32;   // Green
const int WARNING_LED = 33;   // Yellow / Orange
const int DANGER_LED  = 14;   // Red

// Emergency
const int SOS_BUTTON = 27;
const int BUZZER     = 26;


// =====================================================
// NEO-6M GPS (Hardware Serial 2)
// =====================================================

const int GPS_RX_PIN = 16;    // ESP32 RX2 connected to NEO-6M TX
const int GPS_TX_PIN = 17;    // ESP32 TX2 connected to NEO-6M RX
const uint32_t GPS_BAUD = 9600;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2

bool gpsFix = false;
bool hasLastKnownLocation = false;
double gpsLatitude = 0.0;
double gpsLongitude = 0.0;
double lastKnownLatitude = 0.0;
double lastKnownLongitude = 0.0;
float gpsSpeedKmh = 0.0;
float gpsAltitudeM = 0.0;
int gpsSatellites = 0;


// =====================================================
// MPU6050 I2C PINS
// =====================================================

const int MPU_SDA = 21;
const int MPU_SCL = 22;

// MPU6050 I2C address
const int MPU6050_ADDR = 0x68;


// =====================================================
// MPU6050 REGISTERS
// =====================================================

const int MPU_PWR_MGMT_1 = 0x6B;
const int MPU_ACCEL_XOUT_H = 0x3B;
const int MPU_WHO_AM_I = 0x75;


// =====================================================
// MPU6050 VARIABLES
// =====================================================

bool mpuConnected = false;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float rollAngle = 0.0;
float pitchAngle = 0.0;
float tiltAngle = 0.0;


// =====================================================
// TILT THRESHOLDS
// =====================================================

const float TILT_WARNING = 10.0;
const float TILT_DANGER  = 20.0;


// =====================================================
// SYSTEM STATUS
// =====================================================

enum SystemStatus {
  NORMAL,
  WARNING,
  DANGER,
  SOS
};

SystemStatus currentStatus = NORMAL;


// =====================================================
// SOS & BUTTON VARIABLES (Debounced Dual Action)
// =====================================================

bool sosActive = false;

// Button debouncing & long-press settings
const unsigned long LONG_PRESS_DURATION = 2000; // 2 seconds hold to reboot
const unsigned long DEBOUNCE_DELAY      = 40;   // 40ms debounce filter

int lastButtonReading      = HIGH;
int stableButtonState      = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long pressStartTime   = 0;
bool isLongPressHandled        = false;
unsigned long lastHoldPrint    = 0;


// =====================================================
// BUZZER FUNCTIONS
// =====================================================

const int TONE_HIGH = 4000;
const int TONE_LOW  = 3000;

unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;


void buzzerOn(int frequency = TONE_LOW) {
  tone(BUZZER, frequency);
}


void buzzerOff() {
  noTone(BUZZER);
  digitalWrite(BUZZER, LOW);
}


// =====================================================
// ALARM SOUND
// =====================================================

void updateAlarmSound() {

  unsigned long currentMillis = millis();


  // ===================================================
  // SOS ALARM
  // ===================================================

  if (currentStatus == SOS) {

    // Fast dual-tone emergency siren

    if (currentMillis - lastBuzzerToggle >= 120) {

      lastBuzzerToggle = currentMillis;

      buzzerState = !buzzerState;

      if (buzzerState) {

        buzzerOn(TONE_HIGH);
        digitalWrite(DANGER_LED, HIGH);

      } 
      else {

        buzzerOn(TONE_LOW);
        digitalWrite(DANGER_LED, LOW);
      }
    }
  }


  // ===================================================
  // DANGER ALARM
  // ===================================================

  else if (currentStatus == DANGER) {

    // Pulsing danger alarm

    if (currentMillis - lastBuzzerToggle >= 200) {

      lastBuzzerToggle = currentMillis;

      buzzerState = !buzzerState;

      if (buzzerState) {

        buzzerOn(TONE_LOW);
        digitalWrite(DANGER_LED, HIGH);

      } 
      else {

        buzzerOff();
        digitalWrite(DANGER_LED, LOW);
      }
    }
  }


  // ===================================================
  // NORMAL / WARNING
  // ===================================================

  else {

    buzzerOff();
  }
}


// =====================================================
// MPU6050 WRITE
// =====================================================

void writeMPURegister(byte reg, byte value) {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}


// =====================================================
// MPU6050 READ
// =====================================================

int16_t readMPURegister16(byte reg) {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(reg);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 2, true);

  int16_t value = Wire.read() << 8 | Wire.read();

  return value;
}


// =====================================================
// INITIALIZE MPU6050
// =====================================================

bool initializeMPU6050() {

  Serial.println();
  Serial.println("Initializing MPU6050...");

  Wire.begin(MPU_SDA, MPU_SCL);

  delay(100);


  // Read WHO_AM_I register

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_WHO_AM_I);

  if (Wire.endTransmission(false) != 0) {

    Serial.println("MPU6050 NOT detected!");
    return false;
  }


  Wire.requestFrom(MPU6050_ADDR, 1, true);

  if (Wire.available()) {

    byte whoAmI = Wire.read();

    Serial.print("MPU6050 WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);

  } 
  else {

    Serial.println("Unable to read MPU6050.");
    return false;
  }


  // Wake MPU6050 from sleep

  writeMPURegister(MPU_PWR_MGMT_1, 0x00);

  delay(100);


  Serial.println("MPU6050 initialized successfully!");

  return true;
}


// =====================================================
// READ MPU6050
// =====================================================

void readMPU6050() {

  // Read raw accelerometer data

  int16_t rawX = readMPURegister16(0x3B);
  int16_t rawY = readMPURegister16(0x3D);
  int16_t rawZ = readMPURegister16(0x3F);


  // Convert to g
  // Default MPU6050 accelerometer range = +/-2g

  accelX = rawX / 16384.0;
  accelY = rawY / 16384.0;
  accelZ = rawZ / 16384.0;


  // ===================================================
  // Calculate Roll
  // ===================================================

  rollAngle = atan2(
    accelY,
    accelZ
  ) * 180.0 / PI;


  // ===================================================
  // Calculate Pitch
  // ===================================================

  pitchAngle = atan2(
    -accelX,
    sqrt(
      accelY * accelY +
      accelZ * accelZ
    )
  ) * 180.0 / PI;


  // ===================================================
  // Determine maximum tilt
  // ===================================================

  float absoluteRoll = abs(rollAngle);
  float absolutePitch = abs(pitchAngle);

  tiltAngle = max(
    absoluteRoll,
    absolutePitch
  );
}


// =====================================================
// PRINT MPU6050 DATA
// =====================================================

void printMPUData() {

  Serial.print("MPU6050 | ");

  Serial.print("X: ");
  Serial.print(accelX, 2);

  Serial.print("g | Y: ");
  Serial.print(accelY, 2);

  Serial.print("g | Z: ");
  Serial.print(accelZ, 2);

  Serial.print("g | Roll: ");
  Serial.print(rollAngle, 1);

  Serial.print("° | Pitch: ");
  Serial.print(pitchAngle, 1);

  Serial.print("° | Tilt: ");
  Serial.print(tiltAngle, 1);

  Serial.println("°");
}


// =====================================================
// INITIALIZE NEO-6M GPS
// =====================================================

void initializeGPS() {

  Serial.println();
  Serial.println("Initializing NEO-6M GPS on Serial2...");

  // Initialize HardwareSerial2: RX on pin 16, TX on pin 17
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println("GPS Serial initialized (RX2: GPIO 16, TX2: GPIO 17, 9600 baud)");
}


// =====================================================
// READ GPS DATA
// =====================================================

void readGPS() {

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    gpsFix = true;
    gpsLatitude  = gps.location.lat();
    gpsLongitude = gps.location.lng();
    lastKnownLatitude  = gpsLatitude;
    lastKnownLongitude = gpsLongitude;
    hasLastKnownLocation = true;
  } 
  else {
    gpsFix = false;
  }

  if (gps.speed.isValid()) {
    gpsSpeedKmh = gps.speed.kmph();
  }

  if (gps.altitude.isValid()) {
    gpsAltitudeM = gps.altitude.meters();
  }

  if (gps.satellites.isValid()) {
    gpsSatellites = gps.satellites.value();
  }
}


// =====================================================
// PRINT GPS DATA
// =====================================================

void printGPSData() {

  Serial.print("GPS | ");

  if (gpsFix) {

    Serial.print("Lat: ");
    Serial.print(gpsLatitude, 6);

    Serial.print("° | Lon: ");
    Serial.print(gpsLongitude, 6);

    Serial.print("° | Speed: ");
    Serial.print(gpsSpeedKmh, 1);

    Serial.print(" km/h | Alt: ");
    Serial.print(gpsAltitudeM, 1);

    Serial.print(" m | Sats: ");
    Serial.println(gpsSatellites);

  } 
  else {

    Serial.print("Acquiring Satellite Lock... | Sats in view: ");
    Serial.println(gpsSatellites);
  }
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(500);


  // ===================================================
  // Configure LEDs
  // ===================================================

  pinMode(POWER_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);

  pinMode(NORMAL_LED, OUTPUT);
  pinMode(WARNING_LED, OUTPUT);
  pinMode(DANGER_LED, OUTPUT);


  // ===================================================
  // Configure SOS and buzzer
  // ===================================================

  pinMode(SOS_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);


  // ===================================================
  // Initial states
  // ===================================================

  digitalWrite(POWER_LED, HIGH);

  digitalWrite(WIFI_LED, LOW);

  digitalWrite(NORMAL_LED, LOW);
  digitalWrite(WARNING_LED, LOW);
  digitalWrite(DANGER_LED, LOW);

  buzzerOff();


  // ===================================================
  // STARTUP MESSAGE
  // ===================================================

  Serial.println();
  Serial.println("========================================");
  Serial.println("       SEAGUARDIAN IoT DEVICE");
  Serial.println("          MEMBER 01 DEVICE");
  Serial.println("========================================");

  Serial.println();
  Serial.println("ESP32 Power: ON");


  // ===================================================
  // INITIALIZE MPU6050
  // ===================================================

  mpuConnected = initializeMPU6050();


  if (mpuConnected) {

    Serial.println("MPU6050 Status: CONNECTED");

  } 
  else {

    Serial.println("MPU6050 Status: ERROR");
    Serial.println("Check VCC, GND, SDA and SCL connections.");
  }


  // ===================================================
  // INITIALIZE NEO-6M GPS
  // ===================================================

  initializeGPS();


  // ===================================================
  // WIFI CONNECTION
  // ===================================================

  Serial.println();
  Serial.println("Connecting to Wi-Fi...");

  WiFi.begin(ssid, password);

  int wifiAttempts = 0;

  const int maxWifiAttempts = 20;


  while (
    WiFi.status() != WL_CONNECTED &&
    wifiAttempts < maxWifiAttempts
  ) {

    digitalWrite(WIFI_LED, HIGH);

    delay(250);

    digitalWrite(WIFI_LED, LOW);

    delay(250);

    Serial.print(".");

    wifiAttempts++;
  }


  // ===================================================
  // WIFI CONNECTION RESULT
  // ===================================================

  if (WiFi.status() == WL_CONNECTED) {

    digitalWrite(WIFI_LED, HIGH);

    Serial.println();
    Serial.println();

    Serial.println("Wi-Fi Connected!");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

  } 
  else {

    digitalWrite(WIFI_LED, LOW);

    Serial.println();
    Serial.println();

    Serial.println("Wi-Fi Connection Failed or Timed Out.");
    Serial.println("Operating in Offline / Local Mode.");
  }


  // ===================================================
  // DEVICE READY
  // ===================================================

  Serial.println();
  Serial.println("SeaGuardian Device Ready!");


  // ===================================================
  // RED LED BOOT TEST
  // ===================================================

  Serial.println("Testing Red LED at boot...");

  digitalWrite(DANGER_LED, HIGH);

  delay(300);

  digitalWrite(DANGER_LED, LOW);


  // ===================================================
  // INITIAL STATUS
  // ===================================================

  if (mpuConnected) {

    setStatus(NORMAL);

  } 
  else {

    setStatus(WARNING);
  }

  Serial.println();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // ===================================================
  // READ GPS CONTINUOUSLY
  // ===================================================

  readGPS();


  // ===================================================
  // CHECK WIFI
  // ===================================================

  if (WiFi.status() == WL_CONNECTED) {

    digitalWrite(WIFI_LED, HIGH);

  } 
  else {

    digitalWrite(WIFI_LED, LOW);
  }


  // ===================================================
  // CHECK SOS
  // ===================================================

  checkSOS();


  // ===================================================
  // SOS HAS HIGHEST PRIORITY
  // ===================================================

  if (sosActive) {

    if (currentStatus != SOS) {

      setStatus(SOS);
    }

    // Periodic SOS telemetry print every 2 seconds
    static unsigned long lastSOSTelemetry = 0;
    if (millis() - lastSOSTelemetry >= 2000) {
      lastSOSTelemetry = millis();

      Serial.println();
      Serial.println("========================================");
      Serial.println("     🚨 EMERGENCY SOS BROADCAST 🚨     ");
      Serial.println("========================================");
      if (mpuConnected) {
        printMPUData();
      }

      if (gpsFix) {
        Serial.print("📍 GPS LOCATION : ");
        Serial.print(gpsLatitude, 6);
        Serial.print("°, ");
        Serial.print(gpsLongitude, 6);
        Serial.println("°");
        Serial.print("🛰️ SATELLITES   : ");
        Serial.print(gpsSatellites);
        Serial.print(" | Speed: ");
        Serial.print(gpsSpeedKmh, 1);
        Serial.print(" km/h | Alt: ");
        Serial.print(gpsAltitudeM, 1);
        Serial.println(" m");
        Serial.print("🗺️ MAP LINK     : https://maps.google.com/?q=");
        Serial.print(gpsLatitude, 6);
        Serial.print(",");
        Serial.println(gpsLongitude, 6);
      } 
      else if (hasLastKnownLocation) {
        Serial.println("📍 GPS STATUS   : Searching live fix...");
        Serial.print("📌 LAST KNOWN   : ");
        Serial.print(lastKnownLatitude, 6);
        Serial.print("°, ");
        Serial.print(lastKnownLongitude, 6);
        Serial.println("°");
        Serial.print("🗺️ LAST MAP LINK: https://maps.google.com/?q=");
        Serial.print(lastKnownLatitude, 6);
        Serial.print(",");
        Serial.println(lastKnownLongitude, 6);
      } 
      else {
        Serial.print("📍 GPS STATUS   : Acquiring Satellite Fix (Sats in view: ");
        Serial.print(gpsSatellites);
        Serial.println(")");
        Serial.println("💡 TIP          : Ensure GPS antenna has clear view of the sky.");
      }
      Serial.println("========================================");
    }
  }


  // ===================================================
  // MPU6050 & SENSOR STATUS
  // ===================================================

  else {

    bool dangerCondition = false;
    bool warningCondition = false;


    // -------------------------------------------------
    // MPU6050
    // -------------------------------------------------

    if (mpuConnected) {

      readMPU6050();


      // -----------------------------------------------
      // DANGER
      // -----------------------------------------------

      if (tiltAngle > TILT_DANGER) {

        dangerCondition = true;
      }


      // -----------------------------------------------
      // WARNING
      // -----------------------------------------------

      else if (tiltAngle > TILT_WARNING) {

        warningCondition = true;
      }
    }


    // -------------------------------------------------
    // MPU6050 NOT CONNECTED
    // -------------------------------------------------

    else {

      warningCondition = true;
    }


    // -------------------------------------------------
    // Periodic Telemetry (MPU6050 + GPS) every 1 second
    // -------------------------------------------------

    static unsigned long lastTelemetryPrint = 0;

    if (millis() - lastTelemetryPrint >= 1000) {

      lastTelemetryPrint = millis();

      if (mpuConnected) {
        printMPUData();
      }
      printGPSData();
      Serial.println("----------------------------------------");
    }


    // =================================================
    // DETERMINE SYSTEM STATUS
    // =================================================

    if (dangerCondition) {

      if (currentStatus != DANGER) {

        setStatus(DANGER);
      }
    }

    else if (warningCondition) {

      if (currentStatus != WARNING) {

        setStatus(WARNING);
      }
    }

    else {

      if (currentStatus != NORMAL) {

        setStatus(NORMAL);
      }
    }
  }


  // ===================================================
  // UPDATE ALARM
  // ===================================================

  updateAlarmSound();


  delay(20);
}


// =====================================================
// SOS BUTTON & BOARD RESET HANDLER
// =====================================================

void checkSOS() {

  // INPUT_PULLUP:
  // Released = HIGH
  // Pressed  = LOW

  int rawReading = digitalRead(SOS_BUTTON);

  // 1. Debounce state transitions
  if (rawReading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  lastButtonReading = rawReading;

  // 2. Only accept changes after reading is stable for DEBOUNCE_DELAY (40ms)
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {

    // Transition detected
    if (rawReading != stableButtonState) {
      stableButtonState = rawReading;

      // Button just pressed down
      if (stableButtonState == LOW) {
        pressStartTime = millis();
        isLongPressHandled = false;
        lastHoldPrint = millis();
        Serial.println();
        Serial.println("[BUTTON] Pressed...");
      }
      // Button released
      else {
        unsigned long heldDuration = millis() - pressStartTime;
        Serial.print("[BUTTON] Released after ");
        Serial.print(heldDuration);
        Serial.println(" ms");

        // If released before 2s and was not handled as a long press -> SOS
        if (!isLongPressHandled && heldDuration >= 40) {
          if (!sosActive) {
            sosActive = true;

            Serial.println();
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("            🚨 SOS ACTIVATED 🚨");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("Emergency signal detected!");
            Serial.println("SOS has HIGHEST PRIORITY.");
            Serial.println();

            setStatus(SOS);
          }
        }
      }
    }
  }

  // 3. While button is held down (stable LOW), monitor hold duration for reset
  if (stableButtonState == LOW && !isLongPressHandled) {
    unsigned long currentHoldDuration = millis() - pressStartTime;

    // Print hold progress feedback every 400ms to Serial Monitor
    if (millis() - lastHoldPrint >= 400) {
      lastHoldPrint = millis();
      Serial.print("⏳ Holding to reset: ");
      Serial.print(currentHoldDuration / 1000.0, 1);
      Serial.println("s / 2.0s");
    }

    // Trigger board reboot once held for >= 2.0 seconds
    if (currentHoldDuration >= LONG_PRESS_DURATION) {
      isLongPressHandled = true;

      Serial.println();
      Serial.println("****************************************");
      Serial.println("   🔄 LONG PRESS DETECTED (>= 2.0s)");
      Serial.println("       REBOOTING ESP32 BOARD NOW...");
      Serial.println("****************************************");

      // Audio & visual feedback before reboot
      buzzerOff();

      digitalWrite(POWER_LED, HIGH);
      digitalWrite(WIFI_LED, HIGH);
      digitalWrite(NORMAL_LED, HIGH);
      digitalWrite(WARNING_LED, HIGH);
      digitalWrite(DANGER_LED, HIGH);

      buzzerOn(3500);
      delay(300);
      buzzerOff();

      digitalWrite(NORMAL_LED, LOW);
      digitalWrite(WARNING_LED, LOW);
      digitalWrite(DANGER_LED, LOW);
      delay(150);

      // Perform hardware/software reset of ESP32
      ESP.restart();
    }
  }
}


// =====================================================
// STATUS CONTROL
// =====================================================

void setStatus(SystemStatus newStatus) {

  currentStatus = newStatus;


  // ===================================================
  // Turn status LEDs OFF
  // ===================================================

  digitalWrite(NORMAL_LED, LOW);
  digitalWrite(WARNING_LED, LOW);
  digitalWrite(DANGER_LED, LOW);


  // Stop previous alarm

  buzzerOff();


  // Reset alarm timing

  lastBuzzerToggle = millis();

  buzzerState = false;


  // ===================================================
  // NORMAL
  // ===================================================

  if (newStatus == NORMAL) {

    digitalWrite(NORMAL_LED, HIGH);


    Serial.println();
    Serial.println("----------------------------------------");
    Serial.println("SYSTEM STATUS: NORMAL");
    Serial.println("All monitored conditions are safe.");
    Serial.print("Current Tilt: ");
    Serial.print(tiltAngle, 1);
    Serial.println(" degrees");
    Serial.println("----------------------------------------");
  }


  // ===================================================
  // WARNING
  // ===================================================

  else if (newStatus == WARNING) {

    digitalWrite(WARNING_LED, HIGH);


    Serial.println();
    Serial.println("----------------------------------------");
    Serial.println("SYSTEM STATUS: WARNING");
    Serial.println("One or more conditions require attention.");

    if (!mpuConnected) {

      Serial.println("WARNING: MPU6050 not detected.");

    } 
    else {

      Serial.print("Tilt angle: ");
      Serial.print(tiltAngle, 1);
      Serial.println(" degrees");

      Serial.println("Warning threshold exceeded.");
    }

    Serial.println("----------------------------------------");
  }


  // ===================================================
  // DANGER
  // ===================================================

  else if (newStatus == DANGER) {

    digitalWrite(DANGER_LED, HIGH);

    // Buzzer will be controlled by updateAlarmSound()


    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("SYSTEM STATUS: DANGER");
    Serial.println("CRITICAL TILT DETECTED!");
    Serial.print("Tilt angle: ");
    Serial.print(tiltAngle, 1);
    Serial.println(" degrees");
    Serial.println("RED LED + BUZZER ACTIVATED");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }


  // ===================================================
  // SOS
  // ===================================================

  else if (newStatus == SOS) {

    digitalWrite(DANGER_LED, HIGH);

    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("     🚨 SYSTEM STATUS: SOS / CRITICAL 🚨");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("RED LED      : FLASHING SIREN");
    Serial.println("BUZZER       : ON (DUAL-TONE LOUD SIREN)");
    Serial.println("SOS PRIORITY : MAXIMUM");
    
    if (gpsFix) {
      Serial.print("📍 SOS LOCATION: ");
      Serial.print(gpsLatitude, 6);
      Serial.print("°, ");
      Serial.print(gpsLongitude, 6);
      Serial.println("°");
      Serial.print("🗺️ GOOGLE MAPS : https://maps.google.com/?q=");
      Serial.print(gpsLatitude, 6);
      Serial.print(",");
      Serial.println(gpsLongitude, 6);
      Serial.print("🛰️ SATELLITES  : ");
      Serial.println(gpsSatellites);
    } 
    else if (hasLastKnownLocation) {
      Serial.println("📍 SOS LOCATION: Live fix searching...");
      Serial.print("📌 LAST KNOWN  : ");
      Serial.print(lastKnownLatitude, 6);
      Serial.print("°, ");
      Serial.print(lastKnownLongitude, 6);
      Serial.println("°");
      Serial.print("🗺️ GOOGLE MAPS : https://maps.google.com/?q=");
      Serial.print(lastKnownLatitude, 6);
      Serial.print(",");
      Serial.println(lastKnownLongitude, 6);
    } 
    else {
      Serial.print("📍 SOS LOCATION: Acquiring satellite fix... (Sats in view: ");
      Serial.print(gpsSatellites);
      Serial.println(")");
      Serial.println("💡 Note        : Location will display automatically once satellites are locked.");
    }
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
}