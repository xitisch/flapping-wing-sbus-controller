/*
 * receiver.ino
 *
 * ESP32-C3 SuperMini firmware for the SBUS *receiver* node (mounted on the
 * robot).
 * Receives channel values from the transmitter over ESP-NOW and forwards
 * them to the robot as an SBUS signal on Serial1 (GPIO4, inverted, 100k 8E2).
 *
 * Failsafe: SBUS starts only after a healthy locked link is verified. If the
 * radio or GUI link fails, UART1 stops and GPIO4 returns to high-impedance so
 * the flight controller handles the condition as missing SBUS frames.
 */

#include <esp_now.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "sbus.h"
#include "esp_now_link.h"

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select an ESP32-C3 board (for example, ESP32C3 Dev Module)."
#endif

// The C3 has UART0 and UART1 only. Keep UART0/native USB available for debug
// output and dedicate UART1 to the inverted SBUS signal. GPIO4 is exposed on
// the SuperMini and avoids its boot, LED, and native-USB pins.
static constexpr int8_t SBUS_TX_PIN = 4;
constexpr uint32_t SBUS_STARTUP_DELAY_MS = 5000;
constexpr uint32_t SBUS_FRAME_INTERVAL_MS = 10;
constexpr uint8_t SBUS_LINK_QUALIFY_PACKETS = 5;
constexpr uint16_t SBUS_MIN_VALUE = 172;
constexpr uint16_t SBUS_NEUTRAL_VALUE = 992;
constexpr uint16_t SBUS_MAX_VALUE = 1811;
// Battery divider: battery+ -- 300 kOhm -- GPIO3 -- 100 kOhm -- GND.
// The ADC pin therefore sees one quarter of the battery voltage. Never connect
// a battery directly to GPIO3.
static constexpr int8_t BATTERY_SENSE_PIN = 3;
static constexpr uint8_t BATTERY_SAMPLE_COUNT = 16;
bfs::SbusTx sbus(&Serial1, -1, SBUS_TX_PIN, true);

// ==================== RECEIVER VALUE TEST ====================
// Comment out the next line after testing to disable continuous test output.
// #define RECEIVER_VALUE_TEST

#ifdef RECEIVER_VALUE_TEST
static constexpr uint32_t RECEIVER_VALUE_TEST_INTERVAL_MS = 250;
#endif
// ================== END RECEIVER VALUE TEST ==================

// ESP-NOW changed its receive callback's first parameter in Arduino-ESP32 3.x.
// A single version-selected alias also keeps Arduino's .ino prototype generator
// from exposing the 3.x-only type when PlatformIO builds with core 2.x.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
using EspNowRecvInfo = esp_now_recv_info_t;
#else
using EspNowRecvInfo = uint8_t;
#endif

uint16_t sbusChannels[16];

// Link-state, updated from the ESP-NOW receive callback
volatile uint32_t lastPacketMs = 0;
volatile bool linkActive = false;
volatile bool remoteFailsafe = true;
volatile bool packetReceived = false;
volatile uint32_t packetsReceived = 0;
// SBUS remains electrically silent until this many consecutive healthy
// packets arrive with CH8 locked. Once active, CH8 may be unlocked normally.
volatile uint8_t healthyLockedPacketCount = 0;
volatile bool sbusStartPending = false;
volatile bool sbusStopPending = false;
volatile bool sbusOutputActive = false;
uint8_t qualifyingTransmitterMac[6] = {};
bool qualifyingTransmitterMacSet = false;
uint8_t controlTransmitterMac[6] = {};
bool controlTransmitterMacSet = false;

// A valid forward packet tells the receiver which transmitter MAC should get
// the unicast telemetry response. Peer setup and sending happen in loop(), not
// inside ESP-NOW's receive callback.
portMUX_TYPE peerMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t pendingTransmitterMac[6] = {};
volatile bool transmitterPeerPending = false;
uint8_t transmitterMac[6] = {};
bool transmitterPeerReady = false;
uint32_t lastStatusSendMs = 0;

// Safe buffer values used while SBUS output is inactive.
void applyFailsafe() {
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = SBUS_NEUTRAL_VALUE;
  }
  sbusChannels[2] = SBUS_MIN_VALUE;  // CH3: throttle to minimum
  sbusChannels[7] = SBUS_MIN_VALUE;  // CH8: throttle lock disarmed
}

uint16_t channelValueToSbus(uint16_t channelValue) {
  const uint16_t clamped = constrain(channelValue, 1000, 2000);
  const uint32_t scaled =
      static_cast<uint32_t>(clamped - 1000) *
      (SBUS_MAX_VALUE - SBUS_MIN_VALUE);
  return SBUS_MIN_VALUE + (scaled + 500) / 1000;
}

bool handlePacket(const uint8_t *source, const uint8_t *data, int len) {
  if (len != sizeof(SbusPacket)) {
    return false;  // ignore malformed / foreign packets
  }
  SbusPacket packet;
  memcpy(&packet, data, sizeof(packet));

  if (controlTransmitterMacSet &&
      memcmp(source, controlTransmitterMac, 6) != 0) {
    return false;
  }
  if (!controlTransmitterMacSet && qualifyingTransmitterMacSet &&
      memcmp(source, qualifyingTransmitterMac, 6) != 0) {
    return false;
  }

  const uint32_t now = millis();
  const bool packetStreamContinuous =
      linkActive && (now - lastPacketMs <= LINK_TIMEOUT_MS);

  remoteFailsafe = packet.failsafe != 0;
  for (int i = 0; i < 16; i++) {
    sbusChannels[i] = channelValueToSbus(packet.ch[i]);
  }
  packetsReceived++;
  lastPacketMs = now;
  linkActive = true;
  packetReceived = true;

  if (remoteFailsafe) {
    healthyLockedPacketCount = 0;
    sbusStartPending = false;
    if (!controlTransmitterMacSet) {
      qualifyingTransmitterMacSet = false;
    }
    if (sbusOutputActive) {
      sbusStopPending = true;
    }
  } else if (!sbusOutputActive) {
    // Starting or restarting SBUS is allowed only from a deliberately locked
    // state. An unlocked packet resets the qualification sequence.
    if (packet.ch[7] == 1000) {
      if (!qualifyingTransmitterMacSet) {
        memcpy(qualifyingTransmitterMac, source, 6);
        qualifyingTransmitterMacSet = true;
        healthyLockedPacketCount = 0;
      }
      if (!packetStreamContinuous) {
        healthyLockedPacketCount = 0;
      }
      if (healthyLockedPacketCount < SBUS_LINK_QUALIFY_PACKETS) {
        healthyLockedPacketCount++;
      }
      if (healthyLockedPacketCount >= SBUS_LINK_QUALIFY_PACKETS) {
        memcpy(controlTransmitterMac, source, 6);
        controlTransmitterMacSet = true;
        sbusStartPending = true;
      }
    } else {
      healthyLockedPacketCount = 0;
      sbusStartPending = false;
    }
  }
  return true;
}

const uint8_t *sourceMac(const EspNowRecvInfo *info) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  return info->src_addr;
#else
  return info;
#endif
}

void onDataRecv(const EspNowRecvInfo *info, const uint8_t *data, int len) {
  const uint8_t *mac = sourceMac(info);
  if (!handlePacket(mac, data, len)) {
    return;
  }

  portENTER_CRITICAL(&peerMux);
  memcpy(pendingTransmitterMac, mac, 6);
  transmitterPeerPending = true;
  portEXIT_CRITICAL(&peerMux);
}

void configureTransmitterPeer() {
  uint8_t candidateMac[6];
  bool pending = false;

  portENTER_CRITICAL(&peerMux);
  if (transmitterPeerPending) {
    memcpy(candidateMac, pendingTransmitterMac, 6);
    transmitterPeerPending = false;
    pending = true;
  }
  portEXIT_CRITICAL(&peerMux);

  if (!pending) {
    return;
  }
  if (transmitterPeerReady) {
    if (memcmp(candidateMac, transmitterMac, 6) == 0) {
      return;
    }
  }

  if (!esp_now_is_peer_exist(candidateMac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, candidateMac, 6);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
      return;
    }
  }

  memcpy(transmitterMac, candidateMac, 6);
  transmitterPeerReady = true;
}

void sendReceiverStatus() {
  if (!transmitterPeerReady) {
    return;
  }

  ReceiverStatusPacket status = {};
  status.magic = RECEIVER_STATUS_MAGIC;
  status.packets_received = packetsReceived;
  uint32_t rawTotal = 0;
  uint32_t millivoltTotal = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    rawTotal += analogRead(BATTERY_SENSE_PIN);
    millivoltTotal += analogReadMilliVolts(BATTERY_SENSE_PIN);
    delayMicroseconds(100);
  }
  status.battery_adc_raw = rawTotal / BATTERY_SAMPLE_COUNT;
  status.battery_pin_mv = millivoltTotal / BATTERY_SAMPLE_COUNT;
  status.version = RECEIVER_STATUS_VERSION;
  const bool controlReady =
      sbusOutputActive && linkActive && !remoteFailsafe;
  status.link_active = controlReady ? 1 : 0;
  status.failsafe = controlReady ? 0 : 1;
  esp_now_send(transmitterMac, reinterpret_cast<uint8_t *>(&status),
               sizeof(status));
}

void writeSbusFrame() {
  if (!sbusOutputActive || !linkActive || remoteFailsafe) {
    return;
  }

  bfs::SbusData sbusData = {};
  for (int i = 0; i < 16; i++) {
    sbusData.ch[i] = sbusChannels[i];
  }
  sbusData.lost_frame = false;
  // Link loss is represented by stopping SBUS entirely. Never place a
  // failsafe-marked frame on the wire because the flight controller handles
  // missing SBUS frames safely but reacts undesirably to this flag.
  sbusData.failsafe = false;
  sbusData.ch17 = false;
  sbusData.ch18 = false;

  if (!sbusOutputActive || !linkActive || remoteFailsafe) {
    return;
  }
  sbus.data(sbusData);
  sbus.Write();
}

void stopSbusOutput(const char *reason) {
  sbusStartPending = false;
  sbusStopPending = false;
  healthyLockedPacketCount = 0;

  if (sbusOutputActive) {
    sbusOutputActive = false;
    Serial1.end();
    pinMode(SBUS_TX_PIN, INPUT);
    applyFailsafe();
    Serial.println(reason);
  } else {
    pinMode(SBUS_TX_PIN, INPUT);
    applyFailsafe();
  }
}

void startSbusOutputIfQualified() {
  if (sbusOutputActive || !sbusStartPending || !linkActive ||
      remoteFailsafe || healthyLockedPacketCount < SBUS_LINK_QUALIFY_PACKETS) {
    return;
  }

  sbusStartPending = false;
  healthyLockedPacketCount = 0;
  sbus.Begin();
  sbusOutputActive = true;
  writeSbusFrame();
  Serial.println("SBUS active - healthy locked link verified");
}

void setup() {
  const uint32_t startupMs = millis();

  // GPIO4 is high-impedance until the startup guard expires. Do not call
  // sbus.Begin() before then, because that enables UART1 on the pin.
  pinMode(SBUS_TX_PIN, INPUT);
  applyFailsafe();

  Serial.begin(115200);
  delay(2000);
  Serial.println("Receiver booted");

  pinMode(BATTERY_SENSE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_SENSE_PIN, ADC_11db);

  // ESP-NOW runs on Wi-Fi in station mode, disconnected from any AP
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Unable to set ESP-NOW Wi-Fi channel");
  }

  uint8_t mac[6] = {};
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
    Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("Unable to read receiver MAC");
  }
  Serial.printf("SBUS output: GPIO%d\n", SBUS_TX_PIN);

  bool espNowReady = false;
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  } else {
    espNowReady = true;
  }

  while (millis() - startupMs < SBUS_STARTUP_DELAY_MS) {
    delay(1);
  }

  // The startup guard has expired, but GPIO4 remains high-impedance until a
  // healthy packet stream arrives with CH8 deliberately locked.
  applyFailsafe();
  linkActive = false;
  remoteFailsafe = true;
  healthyLockedPacketCount = 0;
  sbusStartPending = false;
  sbusStopPending = false;
  sbusOutputActive = false;

  if (espNowReady) {
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW receiver ready");
    Serial.println("SBUS silent - waiting for healthy locked link");
  }
}

void loop() {
  configureTransmitterPeer();

  // A host-side failsafe request stops the UART instead of transmitting an
  // SBUS frame with the failsafe bit set.
  if (sbusStopPending || (sbusOutputActive && remoteFailsafe)) {
    stopSbusOutput("Host failsafe - SBUS output stopped");
  }

  // A radio timeout also returns GPIO4 to a high-impedance input. The flight
  // controller's missing-SBUS behavior is the verified safe response.
  if (linkActive && (millis() - lastPacketMs > LINK_TIMEOUT_MS)) {
    linkActive = false;
    remoteFailsafe = true;
    stopSbusOutput("Link lost - SBUS output stopped");
  }

  startSbusOutputIfQualified();

  // Debug: handle a newly received ESP-NOW packet.
  if (packetReceived) {
    packetReceived = false;

// ==================== RECEIVER VALUE TEST ====================
#ifdef RECEIVER_VALUE_TEST
    // Print the latest received values at 4 Hz. Printing from loop() keeps
    // serial I/O out of the high-priority ESP-NOW receive callback.
    static uint32_t lastTestPrintMs = 0;
    if (millis() - lastTestPrintMs >= RECEIVER_VALUE_TEST_INTERVAL_MS) {
      lastTestPrintMs = millis();
      Serial.print("[RECEIVER TEST] CH1="); Serial.print(sbusChannels[0]);
      Serial.print(" CH2=");               Serial.print(sbusChannels[1]);
      Serial.print(" CH3=");               Serial.print(sbusChannels[2]);
      Serial.print(" CH5=");               Serial.print(sbusChannels[4]);
      Serial.print(" CH6=");               Serial.print(sbusChannels[5]);
      Serial.print(" CH8=");               Serial.print(sbusChannels[7]);
      Serial.print(" FAILSAFE=");          Serial.println(remoteFailsafe ? "YES" : "NO");
    }
#else
    // Normal debug behavior: print only when one of the used channels changes.
    static uint16_t lastPrinted[16] = {0};
    int dbg[] = {0, 1, 2, 4, 5, 7};  // CH1,CH2,CH3,CH5,CH6,CH8
    bool changed = false;
    for (int k = 0; k < 6; k++)
      if (sbusChannels[dbg[k]] != lastPrinted[dbg[k]]) changed = true;

    if (changed) {
      Serial.print("RX  yaw(CH1):");    Serial.print(sbusChannels[0]);
      Serial.print("  pitch(CH2):");    Serial.print(sbusChannels[1]);
      Serial.print("  thr(CH3):");      Serial.print(sbusChannels[2]);
      Serial.print("  trim1(CH5):");    Serial.print(sbusChannels[4]);
      Serial.print("  trim2(CH6):");    Serial.print(sbusChannels[5]);
      Serial.print("  thr_lock(CH8):"); Serial.println(sbusChannels[7]);
      for (int k = 0; k < 6; k++) lastPrinted[dbg[k]] = sbusChannels[dbg[k]];
    }
#endif
// ================== END RECEIVER VALUE TEST ==================
  }

  writeSbusFrame();

  if (millis() - lastStatusSendMs >= RECEIVER_STATUS_INTERVAL_MS) {
    lastStatusSendMs = millis();
    sendReceiverStatus();
  }

  delay(SBUS_FRAME_INTERVAL_MS);
}
