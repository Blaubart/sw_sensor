/** *****************************************************************************
 * @file    data_logger.cpp
 * @author  Klaus Schaefer,  some adaptions by Maximilian Betz
 * @brief   data logging to uSD
 ******************************************************************************/

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "common.h"
#include "ascii_support.h"
//#include "git-commit-version.h"  /* generated by a git post commit hook in /githooks. */
static ROM char currentGitHash[] = "NIX"; //GIT_COMMIT_HASH;

#if RUN_DATA_LOGGER

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

#define BUFSIZE 2048 // bytes
#define RESERVE 512
static uint8_t __ALIGNED(BUFSIZE) buffer[BUFSIZE + RESERVE];
COMMON static char filename[25];

void
data_logger_runnable (void*)
{
  HAL_SD_DeInit (&hsd);
  delay (2000); //TODO: Quick consecutive resets cause SD Card to hang. This improved but does not fix the situation. Might require switching sd card power

  FRESULT fresult;
  FIL fp;

  // wait until sd card is detected
  while (!BSP_PlatformIsDetected ())
    {
      delay (1000);
    }
  delay (500); // wait until card is plugged correctly

  fresult = f_mount (&fatfs, "", 0);

  if (fresult != FR_OK)
    suspend (); // give up, logger can not work

#ifdef INFILE // SIL simulation requested

  strcpy( filename, "simout.f94");

  FIL infile;

  fresult = f_open(&infile, INFILE, FA_READ);
  if (fresult != FR_OK)
  {
	    asm("bkpt 0");
  }

#else
  // wait until a GNSS timestamp is available.
  while (output_data.c.year == 0)
    {
      delay (100); /* klaus: this bad guy has implemented a spinlock (max) */
    }

  // generate filename based on timestamp
  int idx = 0;
  itoa (2000 + output_data.c.year, filename, 10);
  while (filename[idx] != 0)
    idx++;
  if (output_data.c.month < 10)
    {
      filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.month, &filename[idx], 10);
  while (filename[idx] != 0)
    idx++;

  if (output_data.c.day < 10)
    {
      filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.day, &filename[idx], 10);
  while (filename[idx] != 0)
    idx++;

  if (output_data.c.hour < 10)
    {
      filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.hour, &filename[idx], 10);
  while (filename[idx] != 0)
    idx++;

  if (output_data.c.minute < 10)
    {
      filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.minute, &filename[idx], 10);
  while (filename[idx] != 0)
    idx++;

  if (output_data.c.second < 10)
    {
      filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.second, &filename[idx], 10);
  while (filename[idx] != 0)
    idx++;

  filename[idx] = '.';
  filename[idx + 1] = 'f';
#if LOG_OBSERVATIONS
#if LOG_COORDINATES
  itoa ((sizeof(coordinates_t) + sizeof(measurement_data_t)) / sizeof(float),
	filename + idx + 2, 10);
#endif
#endif
#if LOG_OUTPUT_DATA
  itoa (sizeof(output_data_t) / sizeof(float), filename + idx + 2, 10);
#endif

#endif // simulation requested

  GPIO_PinState led_state = GPIO_PIN_RESET;

  uint32_t writtenBytes = 0;
  uint8_t *buf_ptr = buffer;

  fresult = f_open (&fp, filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    suspend (); // give up, logger can not work

  int32_t sync_counter=0;

#ifdef INFILE // simulation at max speed
  while(true)
    {
      UINT bytesread;
      fresult = f_read(&infile, (void *)&output_data, sizeof(output_data), &bytesread);
      ASSERT( (fresult == FR_OK) && (bytesread == sizeof(output_data)));

      void sync_communicator (void);
      sync_communicator (); // comes from the sensors if not simulated
      delay( 1);
#else
      // logging loop @ 100 Hz
   for (synchronous_timer t (10); true; t.sync ())
    {
#endif

#if LOG_OBSERVATIONS
      memcpy (buf_ptr, (uint8_t*) &output_data.m, sizeof(measurement_data_t));
      buf_ptr += sizeof(measurement_data_t);
#endif
#if LOG_COORDINATES
      memcpy (buf_ptr, (uint8_t*) &(output_data.c), sizeof(coordinates_t));
      buf_ptr += sizeof(coordinates_t);
#endif
#if LOG_OUTPUT_DATA
      memcpy (buf_ptr, (uint8_t*) &(output_data), sizeof(output_data));
      buf_ptr += sizeof(output_data);
#endif
      if (buf_ptr < buffer + BUFSIZE)
	continue; // buffer only filled partially

      fresult = f_write (&fp, buffer, BUFSIZE, (UINT*) &writtenBytes); /*Shall return FR_DENIED if disk is full.*/
      ASSERT((fresult == FR_OK) && (writtenBytes == BUFSIZE)); /* Returns writtenBytes = 0 if disk is full. */
      /* TODO: decide what to do if disk is full.  Simple: Stop Logging.  Better: Remove older files until e.g. 1GB is free
       at startup. FATFS configuration is not up to that. */

      uint32_t rest = buf_ptr - (buffer + BUFSIZE);
      memcpy (buffer, buffer + BUFSIZE, rest);
      buf_ptr = buffer + rest;

      if( ++sync_counter >= 16)
	{
	      f_sync (&fp);
	      sync_counter = 0;
#if uSD_LED_STATUS
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, led_state);
      led_state = led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
	}

    }
}

#define STACKSIZE 1024
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
  { data_logger_runnable, "LOGGER",
  STACKSIZE, 0,
  LOGGER_PRIORITY + portPRIVILEGE_BIT, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 },
      { 0, 0, 0 } } };

RestrictedTask data_logger (p);

#endif
