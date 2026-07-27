/*
 * transmitter.ino
 *
 * ESP32 firmware for the SBUS *transmitter* node (tethered to the PC).
 *
 * Receives channel values from the host GUI over USB serial, then forwards
 * them to the receiver node over ESP-NOW. This replaces the host-side leg of
 * the old wired bridge; the SBUS output now lives on the receiver node.
 *
 * Link:   host --(USB serial)--> transmitter --(ESP-NOW)--> receiver
 *
 * Serial packet format (host -> ESP32):
 *   <CH1,CH2,CH3,CH5,CH6,CH8>
 *   Values are host control units in the range [1000, 2000], not PWM pulse
 *   durations. The receiver maps them to raw SBUS counts.
 *
 * ESP-NOW payload: SbusPacket (see esp_now_link.h), 16 channel values.
 *
 * By default packets are broadcast (FF:FF:FF:FF:FF:FF) so the link works
 * without knowing the receiver's MAC. To pin the link to one receiver, set
 * RECEIVER_MAC below to the address printed by the receiver on boot.
 *
 * Dependencies: esp_now / WiFi  (bundled with the ESP32 Arduino core)
 */

#include <esp_now.h>
#include <esp_arduino_version.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "esp_now_link.h"

// Destination MAC. Broadcast by default; replace with the receiver's MAC
// (printed over serial when the receiver boots) to pin the link.
uint8_t RECEIVER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Resend the latest channel values at least this often (ms). The GUI sends its
// own heartbeat; if that heartbeat disappears, HOST_TIMEOUT_MS locks CH8 and
// marks the outgoing wireless packet as failsafe.
static const uint32_t HEARTBEAT_MS = 50;
static const uint32_t HOST_TIMEOUT_MS = 500;
static const uint8_t HOST_PROTOCOL_VERSION = 3;

uint16_t userChannels[16];
uint32_t controlSequence = 0;
uint32_t lastSendMs = 0;
uint32_t lastHostPacketMs = 0;
bool hostPacketSeen = false;
bool hostFailsafe = true;
bool espNowReady = false;

// ESP-NOW changed its receive callback's first parameter in Arduino-ESP32 3.x.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
using EspNowRecvInfo = esp_now_recv_info_t;
#else
using EspNowRecvInfo = uint8_t;
#endif

void printIdentity() {
  Serial.printf(
      "DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:%u;RADIO:%u\n",
      HOST_PROTOCOL_VERSION, espNowReady ? 1 : 0);
}

void printPong() {
  Serial.printf(
      "PONG;PROTOCOL:%u;HOST_FAILSAFE:%u;RADIO:%u;UPTIME_MS:%lu\n",
      HOST_PROTOCOL_VERSION, hostFailsafe ? 1 : 0, espNowReady ? 1 : 0,
      static_cast<unsigned long>(millis()));
}

portMUX_TYPE receiverStatusMux = portMUX_INITIALIZER_UNLOCKED;
ReceiverStatusPacket pendingReceiverStatus = {};
uint8_t pendingReceiverStatusMac[6] = {};
volatile bool receiverStatusPending = false;

const uint8_t *sourceMac(const EspNowRecvInfo *info) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  return info->src_addr;
#else
  return info;
#endif
}

void onDataRecv(const EspNowRecvInfo *info, const uint8_t *data, int len) {
  if (len != sizeof(ReceiverStatusPacket)) {
    return;
  }

  ReceiverStatusPacket status;
  memcpy(&status, data, sizeof(status));
  if (status.magic != RECEIVER_STATUS_MAGIC) {
    return;
  }
  if (status.version != RECEIVER_STATUS_VERSION) {
    return;
  }

  portENTER_CRITICAL(&receiverStatusMux);
  pendingReceiverStatus = status;
  memcpy(pendingReceiverStatusMac, sourceMac(info), 6);
  receiverStatusPending = true;
  portEXIT_CRITICAL(&receiverStatusMux);
}

void printReceiverStatus() {
  ReceiverStatusPacket status;
  uint8_t source[6];
  bool pending = false;

  portENTER_CRITICAL(&receiverStatusMux);
  if (receiverStatusPending) {
    status = pendingReceiverStatus;
    memcpy(source, pendingReceiverStatusMac, 6);
    receiverStatusPending = false;
    pending = true;
  }
  portEXIT_CRITICAL(&receiverStatusMux);

  if (!pending) {
    return;
  }

  Serial.printf(
      "RECEIVER_STATUS mac=%02X:%02X:%02X:%02X:%02X:%02X packets=%lu "
      "sequence=%lu link=%u failsafe=%u "
      "ch1=%u ch2=%u ch3=%u ch5=%u ch6=%u ch8=%u "
      "battery_raw=%u battery_pin_mv=%u\n",
      source[0], source[1], source[2], source[3], source[4], source[5],
      static_cast<unsigned long>(status.packets_received),
      static_cast<unsigned long>(status.last_sequence), status.link_active,
      status.failsafe, status.applied_ch[0], status.applied_ch[1],
      status.applied_ch[2], status.applied_ch[3], status.applied_ch[4],
      status.applied_ch[5], status.battery_adc_raw, status.battery_pin_mv);
}

void advanceControlSequence() {
  controlSequence++;
  if (controlSequence == 0) {
    controlSequence = 1;  // Reserve zero for "no PC command accepted yet".
  }
}

void sendChannels() {
  SbusPacket packet;
  for (int i = 0; i < 16; i++) {
    packet.ch[i] = userChannels[i];
  }
  packet.sequence = controlSequence;
  packet.failsafe = hostFailsafe ? 1 : 0;
  esp_now_send(RECEIVER_MAC, (uint8_t *)&packet, sizeof(packet));
  lastSendMs = millis();
}

bool parseHostCommand(const String &command, uint16_t values[6]) {
  if (!command.startsWith("<") || !command.endsWith(">")) {
    return false;
  }

  String data = command.substring(1, command.length() - 1);
  int startIndex = 0;

  for (int i = 0; i < 6; i++) {
    int commaIndex = data.indexOf(',', startIndex);
    if ((i < 5 && commaIndex == -1) || (i == 5 && commaIndex != -1)) {
      return false;
    }

    int endIndex = (commaIndex == -1) ? data.length() : commaIndex;
    String token = data.substring(startIndex, endIndex);
    token.trim();
    if (token.length() == 0) {
      return false;
    }
    for (unsigned int j = 0; j < token.length(); j++) {
      if (!isDigit(token[j])) {
        return false;
      }
    }

    values[i] = constrain(token.toInt(), 1000, 2000);
    startIndex = endIndex + 1;
  }
  return true;
}

void engageHostFailsafe() {
  hostPacketSeen = false;
  hostFailsafe = true;
  userChannels[7] = 1000;  // CH8 lock; preserve CH3 as requested
  advanceControlSequence();
  sendChannels();
  Serial.println("GUI connection lost - CH8 locked and failsafe engaged");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);

  // Default all channels to neutral
  for (int i = 0; i < 16; i++) {
    userChannels[i] = 1500;
  }

  // Override channels that need non-neutral safe defaults
  userChannels[0] = 1500;  // CH1: yaw      (neutral)
  userChannels[1] = 1500;  // CH2: pitch    (neutral)
  userChannels[2] = 1000;  // CH3: throttle (minimum - do not arm at neutral)
  userChannels[4] = 1500;  // CH5: trim 1   (neutral)
  userChannels[5] = 1500;  // CH6: trim 2   (neutral)
  userChannels[7] = 1000;  // CH8: throttle lock (locked / disarmed)

  // ESP-NOW runs on Wi-Fi in station mode, disconnected from any AP
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("Unable to set ESP-NOW Wi-Fi channel");
  }
  uint8_t mac[6] = {};
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
    Serial.printf("Transmitter MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.println("Unable to read transmitter MAC");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    printIdentity();
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  // Register the receiver (or broadcast address) as a peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    printIdentity();
    return;
  }
  espNowReady = true;
  Serial.println("ESP-NOW transmitter ready");
  printIdentity();
}

void loop() {
  printReceiverStatus();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "IDENTIFY") {
      printIdentity();
    } else if (command == "PING") {
      printPong();
    } else {
      uint16_t incoming[6];
      if (!parseHostCommand(command, incoming)) {
        return;
      }
      lastHostPacketMs = millis();
      hostPacketSeen = true;

      // After a host timeout, do not automatically re-arm from a stale GUI
      // state. The GUI must send CH8=1000 once before CH8=2000 is accepted.
      if (hostFailsafe && incoming[5] == 1000) {
        hostFailsafe = false;
        Serial.println("GUI connection restored - failsafe cleared while CH8 locked");
      } else if (hostFailsafe) {
        incoming[5] = 1000;
      }

      // Map the six PC values to their non-contiguous SBUS indices.
      bool changed = false;
      for (int i = 0; i < CONTROL_CHANNEL_COUNT; i++) {
        const uint8_t target = CONTROL_CHANNEL_INDICES[i];
        if (userChannels[target] != incoming[i]) {
          changed = true;
        }
        userChannels[target] = incoming[i];
      }

      // A new sequence lets receiver telemetry confirm this accepted command.
      // Heartbeat retransmissions keep this value until another command or
      // transmitter-side failsafe change occurs.
      advanceControlSequence();

      // Push the updated values out over ESP-NOW immediately.
      sendChannels();

      // Do not flood USB serial with unchanged GUI heartbeat packets.
      if (changed) {
        Serial.print("yaw(CH1):");      Serial.println(userChannels[0]);
        Serial.print("pitch(CH2):");    Serial.println(userChannels[1]);
        Serial.print("throttle(CH3):"); Serial.println(userChannels[2]);
        Serial.print("trim1(CH5):");    Serial.println(userChannels[4]);
        Serial.print("trim2(CH6):");    Serial.println(userChannels[5]);
        Serial.print("thr_lock(CH8):"); Serial.println(userChannels[7]);
        Serial.println();
      }
    }
  }

  if (hostPacketSeen && (millis() - lastHostPacketMs > HOST_TIMEOUT_MS)) {
    engageHostFailsafe();
  }

  // ESP-NOW heartbeat: also carries the host-failsafe state to the receiver.
  if (millis() - lastSendMs >= HEARTBEAT_MS) {
    sendChannels();
  }
}
