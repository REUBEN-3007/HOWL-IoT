#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- LoRa SX1278 Pin Definitions (VSPI) ---
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2
#define LORA_FREQ  433E6

// --- Peripherals & Pin Definitions ---
const int BUTTON_PIN = 4;
#define GPS_RX_PIN 16 // ESP32 RX2 connected to GPS TX
#define GPS_TX_PIN 17 // ESP32 TX2 connected to GPS RX

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

unsigned long packetCounter = 0;

// --- Last Known Good (LKG) GPS Buffer ---
struct GPSBuffer {
  double latitude = 0.0;
  double longitude = 0.0;
  float speedKmh = 0.0;
  uint32_t satellites = 0;
  unsigned long lastFixTime = 0;
  bool hasFix = false;
} lkgGps;

// --- Crash Detection Thresholds ---
const float IMPACT_THRESHOLD_G = 3.0;
const float TILT_ANGLE_THRESHOLD_DEG = 65.0;
const unsigned long TILT_SUSTAIN_MS = 2500;
const unsigned long CRASH_COOLDOWN_MS = 6000;

unsigned long tiltStartTime = 0;
bool isTilted = false;
unsigned long lastCrashAlertTime = 0;

// --- Snappy Button Timing ---
const unsigned long DOUBLE_CLICK_GAP_MS = 250; // Rapid window for double-click
const unsigned long DEBOUNCE_DELAY_MS = 20;

int lastButtonState = HIGH;
int buttonState = HIGH;
unsigned long lastDebounceTime = 0;
int clickCount = 0;
unsigned long firstClickTime = 0;

// Continuously drain incoming serial bytes to prevent dropped NMEA sentences
void feedGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lkgGps.latitude = gps.location.lat();
    lkgGps.longitude = gps.location.lng();
    lkgGps.speedKmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    lkgGps.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    lkgGps.lastFixTime = millis();
    lkgGps.hasFix = true;
  }
}

void sendLoRaPacket(const char* eventType, const char* detail) {
  feedGPS();
  packetCounter++;

  char payload[160];
  const char* fixStatus = lkgGps.hasFix ? "LKG" : "NO_FIX";
  if (lkgGps.hasFix && (millis() - lkgGps.lastFixTime < 4000)) {
    fixStatus = "LIVE";
  }

  snprintf(payload, sizeof(payload),
           "%lu,%s,%s,%.6f,%.6f,%.1f,%u,%s",
           packetCounter,
           eventType,
           detail,
           lkgGps.latitude,
           lkgGps.longitude,
           lkgGps.speedKmh,
           lkgGps.satellites,
           fixStatus);

  // ASYNC TRANSMISSION: 'true' offloads transmission to SX1278 hardware
  // so the CPU does not freeze and GPS/Button inputs never lag.
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket(true);

  Serial.println();
  Serial.print(F("📻 [LORA TX #"));
  Serial.print(packetCounter);
  Serial.print(F("] Payload: "));
  Serial.println(payload);

  if (strcmp(fixStatus, "LIVE") == 0) {
    Serial.print(F("   ↳ 📍 Fix: LIVE | Sats: "));
    Serial.println(lkgGps.satellites);
  } else if (strcmp(fixStatus, "LKG") == 0) {
    unsigned long ageSec = (millis() - lkgGps.lastFixTime) / 1000;
    Serial.print(F("   ↳ ⚠️ Fix: Last Known Good ("));
    Serial.print(ageSec);
    Serial.println(F("s ago)"));
  } else {
    Serial.println(F("   ↳ ❌ Fix: Acquiring satellites..."));
  }
}

void triggerCrashAlert(const char* reason) {
  unsigned long now = millis();
  if (now - lastCrashAlertTime > CRASH_COOLDOWN_MS) {
    lastCrashAlertTime = now;
    Serial.println();
    Serial.println(F("*********************************************"));
    Serial.print(F("🚨 [CRASH DETECTED] "));
    Serial.println(reason);
    Serial.println(F("*********************************************"));

    sendLoRaPacket("CRASH", reason);
  }
}

void checkCrashSensors() {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) return;

  if (isnan(a.acceleration.x) || isnan(a.acceleration.y) || isnan(a.acceleration.z)) {
    return;
  }

  float totalAccMps2 = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z);
  float totalG = totalAccMps2 / 9.80665;

  if (totalG < 0.1 || totalG > 25.0) return;

  // 1. High-G Impact
  if (totalG >= IMPACT_THRESHOLD_G) {
    char reasonBuf[32];
    snprintf(reasonBuf, sizeof(reasonBuf), "Impact_%.1fG", totalG);
    triggerCrashAlert(reasonBuf);
    return;
  }

  // 2. Rollover / Sustained Tilt
  float zRatio = constrain(a.acceleration.z / totalAccMps2, -1.0, 1.0);
  float tiltAngle = acos(fabs(zRatio)) * 180.0 / PI;

  if (tiltAngle >= TILT_ANGLE_THRESHOLD_DEG) {
    if (!isTilted) {
      isTilted = true;
      tiltStartTime = millis();
    } else if (millis() - tiltStartTime >= TILT_SUSTAIN_MS) {
      char reasonBuf[32];
      snprintf(reasonBuf, sizeof(reasonBuf), "Tilt_%.0fDeg", tiltAngle);
      triggerCrashAlert(reasonBuf);
    }
  } else {
    isTilted = false;
  }
}

void checkButton() {
  int reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        clickCount++;
        if (clickCount == 1) {
          firstClickTime = millis();
        }
      }
    }
  }
  lastButtonState = reading;

  if (clickCount > 0 && (millis() - firstClickTime > DOUBLE_CLICK_GAP_MS)) {
    if (clickCount == 1) {
      Serial.println(F("\n⛽ [BUTTON] 1 Click -> REFUEL"));
      sendLoRaPacket("REFUEL", "DriverRequest");
    } else if (clickCount >= 2) {
      Serial.println(F("\n🚦 [BUTTON] 2 Clicks -> TRAFFIC"));
      sendLoRaPacket("TRAFFIC", "RoadCongestion");
    }
    clickCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=============================================="));
  Serial.println(F("🚀 NODE A: ASYNC LORA SENDER READY"));
  Serial.println(F("=============================================="));

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // 1. MPU-6050 with hardware I2C timeout protection
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  if (!mpu.begin()) {
    Serial.println(F("❌ ERROR: MPU-6050 not detected!"));
    while (1) delay(500);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println(F("✅ MPU-6050 Initialized."));

  // 2. Hardware GPS Serial
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println(F("✅ GPS Serial Initialized."));

  // 3. SX1278 SPI LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("❌ ERROR: Starting LoRa failed!"));
    while (1) delay(500);
  }

  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println(F("✅ SX1278 LoRa Initialized (Async Mode)."));
  Serial.println(F("Ready: Monitoring active. Tap button or induce tilt/impact."));
}

void loop() {
  feedGPS();
  checkCrashSensors();
  checkButton();
  delay(5); // Ultra-light 5ms cycle keeps polling cadence high
}