#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// --- LoRa SX1278 Pin Definitions (VSPI) ---
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2
#define LORA_FREQ  433E6 // Matches Node A

void parseAndDisplayPacket(String packet, int rssi, float snr) {
  Serial.println(F("\n=============================================="));
  Serial.print(F("📥 [PACKET RECEIVED] RSSI: "));
  Serial.print(rssi);
  Serial.print(F(" dBm | SNR: "));
  Serial.print(snr);
  Serial.println(F(" dB"));
  Serial.print(F("Raw Payload: "));
  Serial.println(packet);
  Serial.println(F("----------------------------------------------"));

  // Format: PKT_ID,EVENT,DETAIL,LAT,LNG,SPEED,SATS,FIX_STATUS (7 commas expected)
  int idx[7];
  idx[0] = packet.indexOf(',');
  for (int i = 1; i < 7; i++) {
    if (idx[i - 1] == -1) {
      idx[i] = -1;
    } else {
      idx[i] = packet.indexOf(',', idx[i - 1] + 1);
    }
  }

  // Strict check: if any of the 7 required commas are missing, reject as malformed
  for (int i = 0; i < 7; i++) {
    if (idx[i] == -1) {
      Serial.println(F("⚠️ Warning: Incomplete/Malformed payload discarded"));
      Serial.println(F("=============================================="));
      return;
    }
  }

  String pktId   = packet.substring(0, idx[0]);
  String event   = packet.substring(idx[0] + 1, idx[1]);
  String detail  = packet.substring(idx[1] + 1, idx[2]);
  String lat     = packet.substring(idx[2] + 1, idx[3]);
  String lng     = packet.substring(idx[3] + 1, idx[4]);
  String speed   = packet.substring(idx[4] + 1, idx[5]);
  String sats    = packet.substring(idx[5] + 1, idx[6]);
  String fixStat = packet.substring(idx[6] + 1);

  if (event == "CRASH") {
    Serial.print(F("🚨 EMERGENCY ALERT: CRASH DETECTED ("));
    Serial.print(detail);
    Serial.println(F(")"));
  } else if (event == "REFUEL") {
    Serial.println(F("⛽ DRIVER ALERT: Refuel Station Needed"));
  } else if (event == "TRAFFIC") {
    Serial.println(F("🚦 DRIVER ALERT: Traffic Congestion Ahead"));
  }

  Serial.print(F("📍 Coordinates: "));
  Serial.print(lat);
  Serial.print(F(", "));
  Serial.println(lng);
  Serial.print(F("⚡ Speed: "));
  Serial.print(speed);
  Serial.print(F(" km/h | Sats: "));
  Serial.print(sats);
  Serial.print(F(" | Fix: "));
  Serial.println(fixStat);
  Serial.println(F("=============================================="));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=============================================="));
  Serial.println(F("📡 NODE B: RECEIVER ACTIVE"));
  Serial.println(F("=============================================="));

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("❌ ERROR: Starting LoRa Receiver failed! Check 3.3V, GND, and SPI pins."));
    while (1) delay(500);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println(F("✅ SX1278 LoRa Receiver Initialized @ 433 MHz."));
  Serial.println(F("Listening for broadcast events from Node A..."));
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incomingPayload = "";
    incomingPayload.reserve(packetSize);

    // Read strictly up to packetSize and filter non-printable characters
    for (int i = 0; i < packetSize; i++) {
      if (LoRa.available()) {
        char c = (char)LoRa.read();
        if (c >= 32 && c <= 126) {
          incomingPayload += c;
        }
      }
    }

    // Flush any leftover hardware FIFO bytes
    while (LoRa.available()) {
      LoRa.read();
    }

    if (incomingPayload.length() > 0) {
      parseAndDisplayPacket(incomingPayload, LoRa.packetRssi(), LoRa.packetSnr());
    }
  }
}