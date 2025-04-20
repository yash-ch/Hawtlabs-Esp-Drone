#include "elrs.h"
#include "imu.h"
#include "config.h"

uint16_t rcChannels[16];
TaskHandle_t crsfTaskHandle = NULL;
volatile bool imu_ready = false;        // Flag to indicate if IMU is ready
volatile bool calibration_mode = false; // Flag to indicate if calibration mode is active
volatile bool armed = true;            // Flag to indicate if the system is armed

void crsfTask(void *pv)
{
  Serial2.begin(CRSF_BAUD_RATE, SERIAL_8N1, RXD0, TXD0);
  Serial.println("CRSF Receiver Task Init");

  for (;;)
  {
    static uint8_t packet[CRSF_MAX_PACKET_SIZE];
    static uint8_t packetIndex = 0;
    static bool inPacket = false;

    while (Serial2.available())
    {
      uint8_t incomingByte = Serial2.read();

      if (!inPacket)
      {
        if (incomingByte == 0xC8 || incomingByte == 0xEE)
        { // Valid CRSF sync bytes
          packetIndex = 0;
          packet[packetIndex++] = incomingByte;
          inPacket = true;
        }
        continue;
      }

      packet[packetIndex++] = incomingByte;

      if (packetIndex >= 2 && packetIndex == packet[1] + 2)
      {
        processCRSFPacket(packet, packetIndex);
        inPacket = false;
        packetIndex = 0;
      }

      if (packetIndex >= CRSF_MAX_PACKET_SIZE)
      {
        packetIndex = 0;
        inPacket = false;
      }
      // if the channel 8 value equal to 2000 and throttle value less than 1000 then we are in calibration mode
      if (rcChannels[8] == 2000 && rcChannels[2] < 1050 && !imu_ready && !calibration_mode && rcChannels[4] < 1500)
      { 
        calibration_mode = true; // Set calibration mode flag
      }
      if (rcChannels[2] < 1050 && rcChannels[4] == 2000 && imu_ready && !armed)
      {
        armed = true;
        wifi_config_mode = true; // disable Wi-Fi config mode if armed
        LoopTimer = micros(); // Reset the loop timer when armed
      }
      else if (rcChannels[4] < 1500)
      {
        armed = false;
        wifi_config_mode = false; // enable Wi-Fi config mode if disarmed        
      }
    }
  }
}

void processCRSFPacket(uint8_t *packet, uint8_t length)
{
  if (length < 24)
    return; // header + type + 22 bytes + CRC

  if (packet[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    return;

  uint8_t *data = &packet[3]; // RC data starts at index 3

  uint32_t bits = 0;
  int bitPos = 0;

  for (int ch = 0; ch < 16; ch++)
  {
    int byteIndex = (bitPos / 8);
    int bitOffset = bitPos % 8;

    // Read next 16 bits to be safe
    bits = data[byteIndex] | (data[byteIndex + 1] << 8) | (data[byteIndex + 2] << 16);

    rcChannels[ch] = (bits >> bitOffset) & 0x7FF;

    bitPos += 11;
  }

  for (int i = 0; i < 16; i++)
  {
    rcChannels[i] = convert_channel_value(rcChannels[i]);
  }
}

uint16_t convert_channel_value(unsigned chan_value)
{
  /*
   *       RC     PWM
   * min  172 ->  988us
   * mid  992 -> 1500us
   * max 1811 -> 2012us
   */
  static constexpr float scale = (2012.f - 988.f) / (1811.f - 172.f);
  static constexpr float offset = 988.f - 172.f * scale;
  return (scale * chan_value) + offset;
}
