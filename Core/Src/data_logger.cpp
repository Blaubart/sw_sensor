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
#include "git-commit-version.h"  /* generated by a git post commit hook in /githooks. */
#include "Linear_Least_Square_Fit.h"
#include "data_structures.h"

COMMON Queue< linear_least_square_result<float>[3] > magnetic_calibration_queue(4);

#if RUN_DATA_LOGGER

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

#define BUFSIZE 2048 // bytes
#define RESERVE 512
static uint8_t __ALIGNED(BUFSIZE) buffer[BUFSIZE + RESERVE];

inline char *append_string( char *target, const char *source)
{
  while( *source)
      *target++ = *source++;
  *target = 0; // just to be sure :-)
  return target;
}

inline char * format_2_digits( char * target, uint32_t data)
{
  data %= 100;
  *target++ = data / 10 + '0';
  *target++ = data % 10 + '0';
  *target = 0; // just be sure string is terminated
  return target;
}
void write_magnetic_calibration_file (const coordinates_t &c)
{
  FRESULT fresult;
  FIL fp;
  char buffer[50];
  char *next = buffer;
  linear_least_square_result<float> data[3];
  int32_t writtenBytes = 0;

  if (false == magnetic_calibration_queue.receive (data, 0))
    return;

  next = format_2_digits(next, c.year);
  next = format_2_digits(next, c.month);
  next = format_2_digits(next, c.day);
  *next++ = '_';
  next = format_2_digits(next, c.hour);
  next = format_2_digits(next, c.minute);
  next = format_2_digits(next, c.second);

  *next++ = data[0].id;

  append_string(next, ".mcl");

  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    return; // silently give up

  for( unsigned i=0; i<3; ++i)
    {
      char *next = buffer;
      next = my_ftoa (next, data[i].a);
      *next++='\t';
      next = my_ftoa (next, data[i].b);
      *next++='\t';
      next = my_ftoa (next, data[i].var_a);
      *next++='\t';
      next = my_ftoa (next, data[i].var_b);
      *next++='\t';
      fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
      if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
        return;
    }
  f_write (&fp, "\r\n", 2, (UINT*) &writtenBytes);
  f_close(&fp);
}

void
data_logger_runnable (void*)
{
  HAL_SD_DeInit (&hsd);
  delay (2000); //TODO: Quick consecutive resets cause SD Card to hang. This improved but does not fix the situation. Might require switching sd card power

  char out_filename[30];
  FRESULT fresult;
  FIL outfile;

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
  FIL infile;
  fresult = f_open(&infile, INFILE, FA_READ);
  if (fresult != FR_OK)
  {
	    asm("bkpt 0");
  }
#endif

#ifdef OUTFILE // SIL simulation requested
  strcpy( out_filename, OUTFILE);
#else
  // wait until a GNSS timestamp is available.
  while (output_data.c.year == 0)
    {
      delay (100); /* klaus: this bad guy has implemented a spinlock (max) */
    }

  // generate filename based on timestamp
  int idx = 0;
  itoa (2000 + output_data.c.year, out_filename, 10);
  while (out_filename[idx] != 0)
    idx++;
  if (output_data.c.month < 10)
    {
      out_filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.month, &out_filename[idx], 10);
  while (out_filename[idx] != 0)
    idx++;

  if (output_data.c.day < 10)
    {
      out_filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.day, &out_filename[idx], 10);
  while (out_filename[idx] != 0)
    idx++;

  if (output_data.c.hour < 10)
    {
      out_filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.hour, &out_filename[idx], 10);
  while (out_filename[idx] != 0)
    idx++;

  if (output_data.c.minute < 10)
    {
      out_filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.minute, &out_filename[idx], 10);
  while (out_filename[idx] != 0)
    idx++;

  if (output_data.c.second < 10)
    {
      out_filename[idx] = '0';
      idx++;
    }
  itoa (output_data.c.second, &out_filename[idx], 10);
  while (out_filename[idx] != 0)
    idx++;

  out_filename[idx] = '.';
  out_filename[idx + 1] = 'f';

#if LOG_OBSERVATIONS
  itoa ( sizeof(measurement_data_t) / sizeof(float),
	out_filename + idx + 2);
#elif LOG_COORDINATES
  itoa ((sizeof(coordinates_t) + sizeof(measurement_data_t)) / sizeof(float),
	out_filename + idx + 2, 10);
#endif

#if LOG_OUTPUT_DATA
  itoa (sizeof(output_data_t) / sizeof(float), out_filename + idx + 2, 10);
#endif

#endif // simulation requested

  GPIO_PinState led_state = GPIO_PIN_RESET;

  uint32_t writtenBytes = 0;
  uint8_t *buf_ptr = buffer;

  fresult = f_open (&outfile, out_filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    suspend (); // give up, logger unable to work

  int32_t sync_counter=0;

#ifdef INFILE
#if MAXSPEED_CALCULATION  // simulation at max speed
  while( true)
#else // simulation in real time
    for (synchronous_timer t (10); true; t.sync ())
#endif
    {
      UINT bytesread;
      fresult = f_read(&infile, (void *)&output_data, IN_DATA_LENGTH*4, &bytesread); // todo PATCH
      if( ! (fresult == FR_OK) && (bytesread == IN_DATA_LENGTH*4)) // probably end of file
	{
	      f_close(&infile);
	      f_close(&outfile);
#if uSD_LED_STATUS
	      while( true)
		{
		  delay(100);
		    HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, led_state);
		    led_state = led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
		  delay(300);
		    HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, led_state);
		    led_state = led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
		}
#else
	      suspend();
#endif
	}

      void sync_communicator (void);
      sync_communicator (); // comes from the sensors if not simulated
      notify_take (true); // wait for synchronization by from communicator

#else
      // logging loop @ 100 Hz
      while( true)
       {
	  notify_take (true); // wait for synchronization by from communicator
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

      fresult = f_write (&outfile, buffer, BUFSIZE, (UINT*) &writtenBytes); /*Shall return FR_DENIED if disk is full.*/
      ASSERT((fresult == FR_OK) && (writtenBytes == BUFSIZE)); /* Returns writtenBytes = 0 if disk is full. */
      /* TODO: decide what to do if disk is full.  Simple: Stop Logging.  Better: Remove older files until e.g. 1GB is free
       at startup. FATFS configuration is not up to that. */

      uint32_t rest = buf_ptr - (buffer + BUFSIZE);
      memcpy (buffer, buffer + BUFSIZE, rest);
      buf_ptr = buffer + rest;

      if( ++sync_counter >= 16)
	{
	      f_sync (&outfile);
	      sync_counter = 0;
#if uSD_LED_STATUS
      HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, led_state);
      led_state = led_state == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
	write_magnetic_calibration_file ( output_data.c);
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

COMMON RestrictedTask data_logger (p);

void sync_logger(void)
  {
    data_logger.notify_give ();
  }

#endif
