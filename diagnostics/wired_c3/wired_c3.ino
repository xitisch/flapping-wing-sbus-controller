/*
 * One-board ESP32-C3 wired diagnostic.
 *
 * Test path:
 *   PC --USB serial--> ESP32-C3 SuperMini --GPIO4 inverted SBUS-->
 *   flight controller
 *
 * This firmware deliberately contains no ESP-NOW code. It accepts the same
 * six host-unit channel values as the maintained controller:
 *   <CH1,CH2,CH3,CH5,CH6,CH8>\n
 *
 * Safety behavior:
 *   - GPIO4 is high-impedance and no SBUS is sent for five seconds after boot.
 *   - SBUS starts only after five consecutive locked commands (CH8=1000).
 *   - Missing USB commands for 500 ms stops UART1 and returns GPIO4 to input.
 *   - Recovery requires another deliberately locked command sequence.
 */

#include <Arduino.h>
#include "sbus.h"

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Select an ESP32-C3 board (for example, ESP32C3 Dev Module)."
#endif

static constexpr int8_t SBUS_TX_PIN = 4;
static constexpr int8_t BATTERY_SENSE_PIN = 3;
static constexpr uint32_t SBUS_STARTUP_DELAY_MS = 5000;
static constexpr uint32_t SBUS_FRAME_INTERVAL_MS = 10;
static constexpr uint32_t HOST_TIMEOUT_MS = 500;
static constexpr uint32_t STATUS_INTERVAL_MS = 250;
static constexpr uint8_t QUALIFYING_LOCKED_COMMANDS = 5;
static constexpr uint8_t BATTERY_SAMPLE_COUNT = 16;
static constexpr uint8_t WIRED_PROTOCOL_VERSION = 1;

static constexpr uint16_t SBUS_MIN_VALUE = 172;
static constexpr uint16_t SBUS_NEUTRAL_VALUE = 992;
static constexpr uint16_t SBUS_MAX_VALUE = 1811;

static constexpr uint8_t CONTROL_CHANNEL_COUNT = 6;
static constexpr uint8_t CONTROL_CHANNEL_INDICES[CONTROL_CHANNEL_COUNT] = {
    0, 1, 2, 4, 5, 7};  // CH1, CH2, CH3, CH5, CH6, CH8

bfs::SbusTx sbus(&Serial1, -1, SBUS_TX_PIN, true);

uint16_t hostChannels[16] = {};
bool sbusOutputActive = false;
bool hostPacketSeen = false;
bool hostFailsafe = true;
uint8_t healthyLockedCommandCount = 0;
uint32_t bootMs = 0;
uint32_t lastHostPacketMs = 0;
uint32_t lastSbusFrameMs = 0;
uint32_t lastStatusMs = 0;
uint32_t commandSequence = 0;

char serialLine[128] = {};
size_t serialLineLength = 0;

void applySafeHostValues() {
  for (int i = 0; i < 16; i++) {
    hostChannels[i] = 1500;
  }
  hostChannels[0] = 1500;  // CH1: yaw neutral
  hostChannels[1] = 1500;  // CH2: pitch neutral
  hostChannels[2] = 1000;  // CH3: throttle minimum
  hostChannels[4] = 1500;  // CH5: trim 1 neutral
  hostChannels[5] = 1500;  // CH6: trim 2 neutral
  hostChannels[7] = 1000;  // CH8: locked
}

uint16_t hostValueToSbus(uint16_t value) {
  const uint16_t clamped = constrain(value, 1000, 2000);
  const uint32_t scaled =
      static_cast<uint32_t>(clamped - 1000) *
      (SBUS_MAX_VALUE - SBUS_MIN_VALUE);
  return SBUS_MIN_VALUE + (scaled + 500) / 1000;
}

void writeSbusFrame() {
  if (!sbusOutputActive || hostFailsafe || !hostPacketSeen) {
    return;
  }

  bfs::SbusData data = {};
  for (int i = 0; i < 16; i++) {
    data.ch[i] = hostValueToSbus(hostChannels[i]);
  }
  data.lost_frame = false;
  data.failsafe = false;
  data.ch17 = false;
  data.ch18 = false;
  sbus.data(data);
  sbus.Write();
}

void stopSbusOutput(const char *reason) {
  healthyLockedCommandCount = 0;
  if (sbusOutputActive) {
    sbusOutputActive = false;
    Serial1.end();
  }
  pinMode(SBUS_TX_PIN, INPUT);
  applySafeHostValues();
  if (reason != nullptr) {
    Serial.println(reason);
  }
}

void startSbusOutput() {
  if (sbusOutputActive || hostFailsafe || !hostPacketSeen ||
      millis() - bootMs < SBUS_STARTUP_DELAY_MS ||
      healthyLockedCommandCount < QUALIFYING_LOCKED_COMMANDS) {
    return;
  }

  healthyLockedCommandCount = 0;
  sbus.Begin();
  sbusOutputActive = true;
  writeSbusFrame();
  lastSbusFrameMs = millis();
  Serial.println("SBUS active - wired locked link verified");
}

void printIdentity() {
  Serial.printf(
      "DEVICE:FLAPPING_WING_WIRED_C3;PROTOCOL:%u\n",
      WIRED_PROTOCOL_VERSION);
}

void printPong() {
  Serial.printf(
      "PONG;PROTOCOL:%u;HOST_FAILSAFE:%u;SBUS:%u;UPTIME_MS:%lu\n",
      WIRED_PROTOCOL_VERSION, hostFailsafe ? 1 : 0,
      sbusOutputActive ? 1 : 0, static_cast<unsigned long>(millis()));
}

void printStatus() {
  uint32_t rawTotal = 0;
  uint32_t millivoltTotal = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    rawTotal += analogRead(BATTERY_SENSE_PIN);
    millivoltTotal += analogReadMilliVolts(BATTERY_SENSE_PIN);
    delayMicroseconds(100);
  }

  const bool linkReady =
      sbusOutputActive && hostPacketSeen && !hostFailsafe;
  Serial.printf(
      "WIRED_STATUS sequence=%lu link=%u failsafe=%u "
      "ch1=%u ch2=%u ch3=%u ch5=%u ch6=%u ch8=%u "
      "battery_raw=%lu battery_pin_mv=%lu\n",
      static_cast<unsigned long>(commandSequence), linkReady ? 1 : 0,
      linkReady ? 0 : 1, hostChannels[0], hostChannels[1],
      hostChannels[2], hostChannels[4], hostChannels[5], hostChannels[7],
      static_cast<unsigned long>(rawTotal / BATTERY_SAMPLE_COUNT),
      static_cast<unsigned long>(millivoltTotal / BATTERY_SAMPLE_COUNT));
}

bool parseChannelCommand(const char *command, uint16_t values[6]) {
  int parsed[6] = {};
  char trailing = '\0';
  const int count = sscanf(
      command, "<%d,%d,%d,%d,%d,%d>%c",
      &parsed[0], &parsed[1], &parsed[2], &parsed[3], &parsed[4],
      &parsed[5], &trailing);
  if (count != 6) {
    return false;
  }
  for (int i = 0; i < CONTROL_CHANNEL_COUNT; i++) {
    if (parsed[i] < 1000 || parsed[i] > 2000) {
      return false;
    }
    values[i] = static_cast<uint16_t>(parsed[i]);
  }
  return true;
}

void handleSerialLine(const char *command) {
  if (strcmp(command, "IDENTIFY") == 0) {
    printIdentity();
    return;
  }
  if (strcmp(command, "PING") == 0) {
    printPong();
    return;
  }

  uint16_t incoming[CONTROL_CHANNEL_COUNT] = {};
  if (!parseChannelCommand(command, incoming)) {
    Serial.println("ERROR:INVALID_COMMAND");
    return;
  }

  lastHostPacketMs = millis();
  hostPacketSeen = true;

  // A timed-out connection cannot restore an unlocked state. The PC must
  // explicitly send CH8=1000 first.
  if (hostFailsafe) {
    if (incoming[5] != 1000) {
      Serial.println("ERROR:LOCK_REQUIRED");
      return;
    }
    hostFailsafe = false;
    healthyLockedCommandCount = 0;
  }

  for (int i = 0; i < CONTROL_CHANNEL_COUNT; i++) {
    hostChannels[CONTROL_CHANNEL_INDICES[i]] = incoming[i];
  }
  commandSequence++;

  if (!sbusOutputActive) {
    if (millis() - bootMs < SBUS_STARTUP_DELAY_MS) {
      healthyLockedCommandCount = 0;
    } else if (hostChannels[7] == 1000) {
      if (healthyLockedCommandCount < QUALIFYING_LOCKED_COMMANDS) {
        healthyLockedCommandCount++;
      }
    } else {
      healthyLockedCommandCount = 0;
    }
    startSbusOutput();
  }
}

void serviceUsbSerial() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r') {
      continue;
    }
    if (character == '\n') {
      if (serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        handleSerialLine(serialLine);
        serialLineLength = 0;
      }
      continue;
    }
    if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = character;
    } else {
      serialLineLength = 0;
      Serial.println("ERROR:COMMAND_TOO_LONG");
    }
  }
}

void setup() {
  bootMs = millis();
  pinMode(SBUS_TX_PIN, INPUT);
  applySafeHostValues();

  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32-C3 wired diagnostic booted");
  printIdentity();

  pinMode(BATTERY_SENSE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_SENSE_PIN, ADC_11db);
  Serial.println("SBUS silent - waiting for startup guard and locked commands");
}

void loop() {
  serviceUsbSerial();

  const uint32_t now = millis();
  if (hostPacketSeen && now - lastHostPacketMs > HOST_TIMEOUT_MS) {
    hostPacketSeen = false;
    hostFailsafe = true;
    stopSbusOutput("Host timeout - SBUS output stopped");
  }

  if (sbusOutputActive &&
      now - lastSbusFrameMs >= SBUS_FRAME_INTERVAL_MS) {
    lastSbusFrameMs = now;
    writeSbusFrame();
  }

  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    printStatus();
  }

  delay(1);
}
