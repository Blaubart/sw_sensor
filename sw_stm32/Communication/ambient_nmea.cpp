#include "ambient_nmea.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// TODO: hier später die echte ESP32-Sendefunktion eintragen
extern void ESP32_send_string(const char *s);

static uint8_t nmea_checksum(const char *sentence_without_dollar)
{
  uint8_t checksum = 0;

  while (*sentence_without_dollar != '\0')
  {
    checksum ^= (uint8_t)(*sentence_without_dollar);
    ++sentence_without_dollar;
  }

  return checksum;
}

void send_PLARB_to_ESP32(float battery_voltage,
                         float outside_temperature,
                         float relative_humidity)
{
  char payload[48];
  char sentence[64];

  snprintf(payload, sizeof(payload),
           "PLARB,%.2f,%.1f,%.1f",
           battery_voltage,
           outside_temperature,
           relative_humidity);

  uint8_t checksum = nmea_checksum(payload);

  snprintf(sentence, sizeof(sentence),
           "$%s*%02X\r\n",
           payload,
           checksum);

  ESP32_send_string(sentence);
}