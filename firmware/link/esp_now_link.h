/*
 * esp_now_link.h
 *
 * Single definition of the ESP-NOW link payload exchanged between the SBUS
 * transmitter and receiver PlatformIO projects.
 */

#ifndef ESP_NOW_LINK_H
#define ESP_NOW_LINK_H

#include <stdint.h>

static const uint8_t CONTROL_CHANNEL_COUNT = 6;
static const uint8_t CONTROL_CHANNEL_INDICES[CONTROL_CHANNEL_COUNT] = {
    0, 1, 2, 4, 5, 7};  // CH1, CH2, CH3, CH5, CH6, CH8

// Payload sent over ESP-NOW. Channel values are host control units [1000, 2000],
// not PWM pulse durations. The receiver maps them to raw SBUS counts.
typedef struct __attribute__((packed)) {
  uint16_t ch[16];
  uint8_t failsafe;
} SbusPacket;

// Receiver-to-transmitter telemetry reports link health and GPIO3 battery
// measurement only. It deliberately does not echo or acknowledge channels.
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t packets_received;
  uint16_t battery_adc_raw;
  uint16_t battery_pin_mv;
  uint8_t version;
  uint8_t link_active;
  uint8_t failsafe;
} ReceiverStatusPacket;

static_assert(sizeof(SbusPacket) == 33, "Unexpected SbusPacket layout");
static_assert(sizeof(ReceiverStatusPacket) == 15,
              "Unexpected ReceiverStatusPacket layout");

static const uint32_t RECEIVER_STATUS_MAGIC = 0x46575354UL;  // FWST
static const uint8_t RECEIVER_STATUS_VERSION = 2;
static const uint32_t RECEIVER_STATUS_INTERVAL_MS = 250;

// Both ESP-NOW nodes must use the same 2.4 GHz Wi-Fi channel.
static const uint8_t ESPNOW_WIFI_CHANNEL = 1;

// If no packet is received within this window the receiver engages failsafe.
static const uint32_t LINK_TIMEOUT_MS = 500;

#endif  // ESP_NOW_LINK_H
