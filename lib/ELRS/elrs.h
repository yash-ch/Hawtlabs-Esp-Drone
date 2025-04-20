#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

extern uint16_t rcChannels[16]; // Channel data
#define CRSF_BAUD_RATE 420000 // CRSF baud rate
#define RXD0 44               // ESP32-S3 RX (connected to Receiver TX)
#define TXD0 43               // ESP32-S3 TX (not used by receiver)
#define CRSF_MAX_PACKET_SIZE 64
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

extern volatile bool calibration_mode; // Flag to indicate if calibration mode is active
extern volatile bool imu_ready;       // Flag to indicate if IMU is ready
extern volatile bool armed;        // Flag to indicate if the system is armed


void crsfTask(void *pv);
void processCRSFPacket(uint8_t *data, uint8_t length);
uint16_t convert_channel_value(unsigned chan_value);
extern TaskHandle_t crsfTaskHandle;