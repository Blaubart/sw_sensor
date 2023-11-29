/** *****************************************************************************
 * @file    	uSD_handler.cpp
 * @brief   	uSD File reader + writer
 * @author  	Dr. Klaus Schaefer,  some adaptions by Maximilian Betz
 * @copyright 	Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 **************************************************************************/

#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "fatfs.h"
#include "common.h"
#include "ascii_support.h"
#include "git-commit-version.h"  /* generated by a git post commit hook in /githooks.  */
#include "Linear_Least_Square_Fit.h"
#include "data_structures.h"
#include "read_configuration_file.h"
#include "communicator.h"
#include "emergency.h"
#include "uSD_handler.h"
#include "watchdog_handler.h"
#include "EEPROM_defaults.h"
#include "magnetic_induction_report.h"

extern Semaphore setup_file_handling_completed;
extern uint32_t UNIQUE_ID[4];
extern bool reset_by_watchdog_requested;

COMMON char *crashfile;
COMMON unsigned crashline;
COMMON bool magnetic_gound_calibration;
COMMON bool dump_sensor_readings;

COMMON Semaphore magnetic_calibration_done;
COMMON magnetic_induction_report_t magnetic_induction_report;

FATFS fatfs;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern int64_t FAT_time; //!< DOS FAT time for file usage

#define MEM_BUFSIZE 2048 // bytes
#define RESERVE 512
static uint8_t __ALIGNED(MEM_BUFSIZE) mem_buffer[MEM_BUFSIZE + RESERVE];

//!< format date and time from sat fix data
char * format_date_time( char * target)
{
  target = format_2_digits( target, output_data.c.year);
  target = format_2_digits( target, output_data.c.month);
  target = format_2_digits( target, output_data.c.day);
  *target ++ = '_';
  target = format_2_digits( target, output_data.c.hour);
  target = format_2_digits( target, output_data.c.minute);
  target = format_2_digits( target, output_data.c.second);
  *target=0;
  return target;
}

extern RestrictedTask uSD_handler_task; // will come downwards ...

//!< write crash dump file and force MPU reset via watchdog
void write_crash_dump( void)
{
  FRESULT fresult;
  FIL fp;
  char buffer[50];
  char *next = buffer;
  UINT writtenBytes = 0;

#if configUSE_TRACE_FACILITY // ************************************************
#include "trcConfig.h"
  vTraceStop(); // don't trace ourselves ...
#endif

  next = format_date_time( buffer);
  next = append_string (next, ".CRASHDUMP");

  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    return;

  next=append_string( buffer, (char*)"Firmware: ");
  next=append_string( next, GIT_TAG_INFO);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"Hardware: ");
  next = utox( next, UNIQUE_ID[0], 8);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, crashfile);
  next=append_string( next, (char*)" Line: ");
  next = my_itoa( next, crashline);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"Task:     ");
  next=append_string( next, pcTaskGetName( (TaskHandle_t)(register_dump.active_TCB)));
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"IPSR:     ");
  next = utox( next, register_dump.IPSR);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"PC:       ");
  next = utox( next, register_dump.stacked_pc);
  newline( next);
  next=append_string( next, (char*)"LR:       ");
  next = utox( next, register_dump.stacked_lr);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"BusFA:    ");
  next = utox(  next, register_dump.Bus_Fault_Address);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"MemA:     ");
  next = utox( next, register_dump.Bad_Memory_Address);
  newline( next);

  next=append_string( next, (char*)"MemFS:    ");
  next = utox( next, register_dump.Memory_Fault_status);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"FPU_S:    ");
  next = utox( next, register_dump.FPU_StatusControlRegister);
  newline( next);

  next=append_string( next, (char*)"UsgFS:    ");
  next = utox( next, register_dump.Usage_Fault_Status_Register);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  next=append_string( buffer, (char*)"HardFS:   ");
  next = utox( next, register_dump.Hard_Fault_Status);
  newline( next);

  f_write (&fp, buffer, next-buffer, &writtenBytes);

  f_close(&fp);

#if configUSE_TRACE_FACILITY // ************************************************

extern RecorderDataType myTraceBuffer;
  next = format_date_time( buffer);
  next = append_string (next, ".bin");
  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    return;

  for( uint8_t *ptr=(uint8_t *)&myTraceBuffer; ptr < (uint8_t *)&myTraceBuffer + sizeof(RecorderDataType); ptr += 2048)
    {
      UINT size = (uint8_t *)&myTraceBuffer + sizeof(RecorderDataType) - ptr;
      if( size > MEM_BUFSIZE)
	size = MEM_BUFSIZE;
      memcpy( mem_buffer, ptr, size);
      fresult = f_write (&fp, (const void *)mem_buffer, size, &writtenBytes);
      if( writtenBytes < size)
	break;
    }
  f_close(&fp);

#endif // ************************************************************************

  // log one complete set of output data independent of data logging status
  next = format_date_time( buffer);
  *next++ = '.';
  *next++  = 'f';
  next = format_2_digits( next, sizeof( output_data_t) / sizeof(float));

  fresult = f_open ( &fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult == FR_OK)
    {
      fresult = f_write (&fp, (uint8_t*) &output_data, sizeof( output_data_t), &writtenBytes);
      f_close( &fp);
    }

  delay(1000); // wait until data has been saved and file is closed (DMA...)

  while( true)
    /* wake watchdog */;
}

bool write_EEPROM_dump( const char * filename)
{
  FRESULT fresult;
  FIL fp;
  char buffer[50];
  char *next = buffer;
  int32_t writtenBytes = 0;

  next = append_string (next, filename);
  next = append_string (next, ".EEPROM");
  *next=0;

  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    return fresult;

  f_write (&fp, GIT_TAG_INFO, strlen(GIT_TAG_INFO), (UINT*) &writtenBytes);
  f_write (&fp, "\r\n", 2, (UINT*) &writtenBytes);
  utox( buffer, UNIQUE_ID[0], 8);
  buffer[8]='\r';
  buffer[9]='\n';
  f_write (&fp, buffer, 10, (UINT*) &writtenBytes);

  for( unsigned index = 1; index < PERSISTENT_DATA_ENTRIES; ++index)
    {
      float value;
      bool result = read_EEPROM_value( PERSISTENT_DATA[index].id, value);
      if( result == HAL_OK)
	{
	  if( PERSISTENT_DATA[index].is_an_angle)
	    value *= 180.0 / M_PI_F; // format it human readable

	  next = buffer;
	  next = format_2_digits(next, PERSISTENT_DATA[index].id);
	  *next++=' ';
	  next = append_string( next, PERSISTENT_DATA[index].mnemonic);
	  next = append_string (next," = ");
	  next = my_ftoa (next, value);
	  *next++='\r';
	  *next++='\n';
	  *next=0;

	  fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
	  if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
	    {
	      f_close(&fp);
	      return fresult; // give up ...
	    }
	}
      }

  f_close(&fp);
  return FR_OK;
}

void write_magnetic_calibration_file ( void)
{
  FRESULT fresult;
  FILINFO filinfo;
  FIL fp;
  char buffer[100];
  char *next = buffer;
  int32_t writtenBytes = 0;

  fresult = f_stat("magnetic", &filinfo);
  if( (fresult != FR_OK) || ((filinfo.fattrib & AM_DIR)==0))
    return; // directory does not exist -> do not write file

  next = append_string( next, "magnetic/");
  next = format_date_time( next);
  append_string(next, ".mcl");

  fresult = f_open (&fp, buffer, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    return; // silently give up

  for( unsigned i=0; i<3; ++i)
    {
      char *next = buffer;
      next = my_ftoa (next, magnetic_induction_report.calibration[i].offset);
      *next++=' ';
      next = my_ftoa (next, magnetic_induction_report.calibration[i].scale);
      *next++=' ';
      next = my_ftoa (next, SQRT( magnetic_induction_report.calibration[i].variance));
      *next++=' ';
      fresult = f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
      if( (fresult != FR_OK) || (writtenBytes != (next-buffer)))
        return;
    }

  next = buffer;
  float3vector induction = magnetic_induction_report.nav_induction;
  for( unsigned i=0; i<3; ++i)
    {
      next = my_ftoa (next, induction[i]);
      *next++=' ';
    }

  next = my_ftoa (next, magnetic_induction_report.nav_induction_std_deviation);
  *next++='\r';
  *next++='\n';

  f_write (&fp, buffer, next-buffer, (UINT*) &writtenBytes);
  f_close(&fp);
}

//!< find software image file and load it if applicable
bool read_software_update(void)
{
  FIL the_file;
  FRESULT fresult;
  UINT bytes_read;
  uint32_t flash_address = 0x80000;
  unsigned status;
  bool last_block_read = false;

  // try to open new software image file
  fresult = f_open (&the_file, (char *)"image.bin", FA_READ);
  if( fresult != FR_OK)
    return false;

  // read first block
  fresult = f_read(&the_file, mem_buffer, MEM_BUFSIZE, &bytes_read);
  if( (fresult != FR_OK) || (bytes_read  < MEM_BUFSIZE))
    {
      f_close(&the_file);
      return false;
    }

  // compare against flash content
  uint32_t *mem_ptr;
  uint32_t *flash_ptr;
  bool image_is_equal = true;

  for( mem_ptr=(uint32_t *)mem_buffer, flash_ptr=(uint32_t *)flash_address; mem_ptr < (uint32_t *)(mem_buffer + MEM_BUFSIZE); ++mem_ptr, ++flash_ptr)
    if(*mem_ptr != *flash_ptr)
      {
	image_is_equal = false;
	break;
      }

  if( image_is_equal)
    return false;

  status = HAL_FLASH_Unlock();
  if(status != HAL_OK)
    return false;

  // for an unknown reason error flags need to be reset
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);

  // erase flash range 0x08080000 - 0x080DFFFF
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  pEraseInit.NbSectors = 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;
  pEraseInit.Sector = FLASH_SECTOR_8;
  status = HAL_FLASHEx_Erase (&pEraseInit, &SectorError);
  if ((status != HAL_OK) || (SectorError != 0xffffffff))
    return false;
  pEraseInit.Sector = FLASH_SECTOR_9;
  status = HAL_FLASHEx_Erase (&pEraseInit, &SectorError);
  if ((status != HAL_OK) || (SectorError != 0xffffffff))
    return false;
  pEraseInit.Sector = FLASH_SECTOR_10;
  status = HAL_FLASHEx_Erase (&pEraseInit, &SectorError);
  if ((status != HAL_OK) || (SectorError != 0xffffffff))
    return false;

  for(;;)
    {
      for( uint32_t * data_pointer = (uint32_t *)mem_buffer; data_pointer < (uint32_t *)( mem_buffer + bytes_read); ++data_pointer)
      {
	      status = HAL_FLASH_Program( TYPEPROGRAM_WORD, flash_address, (uint64_t) *data_pointer);
	      if(status != HAL_OK)
		  break;
	      flash_address += sizeof(uint32_t);
      }

      if( last_block_read)
	{
	  HAL_FLASH_Lock();
	  return true;
	}

      fresult = f_read(&the_file, mem_buffer, MEM_BUFSIZE, &bytes_read);
      if( fresult != FR_OK)
	{
	  f_close(&the_file);
	  break;
	}

      if( bytes_read < MEM_BUFSIZE)
	{
	  f_close(&the_file);
	  last_block_read = true;
	}
    }
  HAL_FLASH_Lock();
  return false;
}

//!< this executable takes care of all uSD reading and writing
void uSD_handler_runnable (void*)
{
  // if no EEPROM data exist: write default values
  // later we will take care of the individual configuration
  if( ! all_EEPROM_parameters_existing())
      write_EEPROM_defaults();

restart:

  HAL_SD_DeInit (&hsd);
  delay (1000);

  // wait max. 10s until sd card is detected
  for( int i=10; i>0 && (! BSP_PlatformIsDetected()); --i)
      delay (1000);
  delay (100); // wait until card is plugged correctly

  if(! BSP_PlatformIsDetected())
    {
      setup_file_handling_completed.signal(); // give up waiting for configuration
      watchdog_activator.signal(); // now start the watchdog

  while(true) // wait until uSD plugged in and restart the uSD handler afterwards
	{
	  delay(1000);
	  if( BSP_PlatformIsDetected())
	    goto restart;
	}
    }

  HAL_StatusTypeDef hresult = HAL_SD_Init (&hsd);
  if( hresult != HAL_OK)
    goto restart;

  FRESULT fresult;
  fresult = f_mount (&fatfs, "", 0);

  if (fresult != FR_OK)
    {
      setup_file_handling_completed.signal();
      watchdog_activator.signal(); // now start the watchdog

      while(true) // wait until uSD UN-plugged
	{
	  delay(1000);
	  if( ! BSP_PlatformIsDetected())
	    break;
	}
      while(true) // wait until uSD plugged in and restart the uSD handler afterwards
	{
	  if( BSP_PlatformIsDetected())
	    goto restart;
	  delay(1000);
	}
    }

  // LED on to signal "uSD active"
  HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);

  if( read_software_update())
      {
      *( ( volatile uint32_t * ) 0xe000ed94 ) = 0; // MPU off
      __asm volatile ( "dsb" ::: "memory" );
      __asm volatile ( "isb" );
      typedef void(*pFunction)(void);
      pFunction copy_function_address = *(pFunction *)0x08001c;
      copy_function_address();
      }

  watchdog_activator.signal(); // now start the watchdog

  read_configuration_file(); // read configuration file if it is present on the SD card
  setup_file_handling_completed.signal();

  delay( 100); // give communicator a moment to initialize

  FIL the_file;

  UINT writtenBytes = 0;
  uint8_t *buf_ptr = mem_buffer;

  fresult = f_open (&the_file, (char *)"magnetic.calibration", FA_READ);
  if( fresult == FR_DISK_ERR)
    goto restart;
  magnetic_gound_calibration = (fresult == FR_OK);
  f_close( &the_file); // as this is just a dummy file

  fresult = f_open (&the_file, (char *)"sensor.readings", FA_READ);
  dump_sensor_readings = (fresult == FR_OK);
  f_close( &the_file); // as this is just a dummy file

  if( magnetic_gound_calibration)
    {
      write_EEPROM_dump( (char *)"before_calibration");
      magnetic_calibration_done.wait();
      write_EEPROM_dump( (char *)"after_calibration");
      f_unlink((char *)"magnetic.calibration");
      while( 1)
	{
	notify_take (true); // wait for synchronization by crash detection
	if( crashfile)
	  write_crash_dump();
	}
    }

  FILINFO filinfo;
  fresult = f_stat("logger", &filinfo);
  if( (fresult != FR_OK) || ((filinfo.fattrib & AM_DIR)==0))
    while( 1)
	{
	notify_take (true); // wait for synchronization by crash detection
	if( crashfile)
	  write_crash_dump();
	}

  // wait until a GNSS timestamp is available.
  while (output_data.c.sat_fix_type == 0)
    {
      if( crashfile)
	write_crash_dump();
      delay (100);
    }

  // generate filename based on timestamp
  char out_filename[30];
  char * next = append_string( out_filename, "logger/");
  next = format_date_time( next);

  write_EEPROM_dump( out_filename); // now we have date+time, start logging

  *next++ = '.';
  *next++  = 'f';
  next = format_2_digits( next, (sizeof(coordinates_t) + sizeof(measurement_data_t)) / sizeof(float));

  fresult = f_open (&the_file, out_filename, FA_CREATE_ALWAYS | FA_WRITE);
  if (fresult != FR_OK)
    {
      while( true)
	{
	notify_take (true); // wait for synchronization by crash detection
	if( crashfile)
	  write_crash_dump();
	}
    }

  int32_t sync_counter=0;

  while( true) // logger loop synchronized by communicator
    {
      notify_take (true); // wait for synchronization by from communicator OR crash detection

      if( crashfile)
	write_crash_dump();

      memcpy (buf_ptr, (uint8_t*) &output_data.m, sizeof(measurement_data_t)+sizeof(coordinates_t));
      buf_ptr += sizeof(measurement_data_t)+sizeof(coordinates_t);

      if (buf_ptr < mem_buffer + MEM_BUFSIZE)
	continue; // buffer only filled partially

      fresult = f_write (&the_file, mem_buffer, MEM_BUFSIZE, &writtenBytes);
      if( ! ((fresult == FR_OK) && (writtenBytes == MEM_BUFSIZE)))
	  {
	    HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
	    while( true)
	      {
	      notify_take (true); // wait for synchronization by crash detection
	      if( crashfile)
		write_crash_dump();
	      }
	  }

      uint32_t rest = buf_ptr - (mem_buffer + MEM_BUFSIZE);
      memcpy (mem_buffer, mem_buffer + MEM_BUFSIZE, rest);
      buf_ptr = mem_buffer + rest;

#if uSD_LED_STATUS
      if( (sync_counter & 0x3) == 0)
	HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_SET);
      else
	HAL_GPIO_WritePin (LED_STATUS1_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
#endif

      if( ++sync_counter >= 16)
	{
	  sync_counter = 0;
	  f_sync (&the_file);
	  if( magnetic_calibration_done.wait( 0))
	    write_magnetic_calibration_file ();
	}
    }
}

#define STACKSIZE (1024*2)
static uint32_t __ALIGNED(STACKSIZE*4) stack_buffer[STACKSIZE];

static TaskParameters_t p =
  { uSD_handler_runnable, "uSD",
  STACKSIZE, 0,
  LOGGER_PRIORITY + portPRIVILEGE_BIT, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void *)0x80f8000, 0x8000, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 } 
      } 
    };

COMMON RestrictedTask uSD_handler_task (p);

extern "C" void sync_logger(void)
  {
    uSD_handler_task.notify_give ();
  }

//!< this function is called synchronously from task context
extern "C" void emergency_write_crashdump( char * file, int line)
  {
  acquire_privileges();
  crashfile=file;
  crashline=line;
  extern void * pxCurrentTCB;
  register_dump.active_TCB = pxCurrentTCB;
  uSD_handler_task.set_priority(configMAX_PRIORITIES - 1); // set it to highest priority
  uSD_handler_task.notify_give();
  suspend();
  }

COMMON bool watchdog_has_been_triggered = false;

//!< helper task to stop everything and launch emergency logging
void kill_amok_running_task( void *)
{
  while ( 0 == register_dump.active_TCB)
	suspend();

  vTaskSuspend( (TaskHandle_t)(register_dump.active_TCB)); // probably the amok running task

  if( watchdog_has_been_triggered)
    watchdog_handler.suspend(); // avoid WWDG reset out of phase if already activated

  uSD_handler_task.set_priority(configMAX_PRIORITIES - 1); // set it to highest priority
  uSD_handler_task.notify_give();
  while( true) // job done
    suspend();
}

RestrictedTask amok_running_task_killer( kill_amok_running_task, "ANTI_AMOK", configMINIMAL_STACK_SIZE, 0, configMAX_PRIORITIES -1);

//!< this function is called in exception context
extern "C" void finish_crash_handling( void)
{
  extern void * pxCurrentTCB;
  register_dump.active_TCB = pxCurrentTCB;

  // remember what happened
  crashfile=(char *)"EXCEPTION";
  crashline=0;

  // triggger error logging
  uSD_handler_task.notify_give_from_ISR();
  amok_running_task_killer.resume_from_ISR();
}

//!< this function is called if the watchdog has been woken up
extern "C" void handle_watchdog_trigger( void)
{
  extern void * pxCurrentTCB;
  register_dump.active_TCB = pxCurrentTCB;

  // remember what happened
  crashfile=(char *)"WATCHDOG";
  crashline=0;

  watchdog_has_been_triggered = true;

  // triggger error logging
  uSD_handler_task.notify_give_from_ISR();
  amok_running_task_killer.resume_from_ISR();
}

void report_magnetic_calibration_has_changed(magnetic_induction_report_t * p_magnetic_induction_report, char)
{
  magnetic_induction_report = *p_magnetic_induction_report;
  magnetic_calibration_done.signal();
}
