#include "SHT35.h"

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

/*
 * SHT35 I2C address:
 * ADDR pin to GND = 0x44
 * ADDR pin to VCC = 0x45
 *
 * HAL expects shifted address
 */
#define SHT35_ADDR (0x44 << 1)

/*
 * High repeatability measurement
 * clock stretching disabled
 */
static const uint8_t measure_cmd[2] = {0x24, 0x00};

bool SHT35_init(void)
{
  /*
   * optional:
   * test if sensor responds
   */
  if (HAL_I2C_IsDeviceReady(&hi2c1,
                            SHT35_ADDR,
                            2,
                            100) == HAL_OK)
  {
    return true;
  }

  return false;
}

static uint8_t SHT35_crc(const uint8_t *data, int len)
{
  uint8_t crc = 0xFF;

  for (int i = 0; i < len; ++i)
  {
    crc ^= data[i];

    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }

  return crc;
}

bool SHT35_read(float &temperature, float &humidity)
{
  uint8_t rx[6];

  /*
   * start measurement
   */
  if (HAL_I2C_Master_Transmit(&hi2c1,
                              SHT35_ADDR,
                              (uint8_t*)measure_cmd,
                              2,
                              100) != HAL_OK)
  {
    return false;
  }

  HAL_Delay(20);

  /*
   * read result
   */
  if (HAL_I2C_Master_Receive(&hi2c1,
                             SHT35_ADDR,
                             rx,
                             6,
                             100) != HAL_OK)
  {
    return false;
  }

  /*
   * CRC check
   */
  if (SHT35_crc(rx, 2) != rx[2])
    return false;

  if (SHT35_crc(&rx[3], 2) != rx[5])
    return false;

  uint16_t raw_temp =
      ((uint16_t)rx[0] << 8) | rx[1];

  uint16_t raw_hum =
      ((uint16_t)rx[3] << 8) | rx[4];

  /*
   * conversion formulas from datasheet
   */
  temperature =
      -45.0f +
      175.0f * ((float)raw_temp / 65535.0f);

  humidity =
      100.0f * ((float)raw_hum / 65535.0f);

  return true;
}