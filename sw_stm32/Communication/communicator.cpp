/***********************************************************************//**
 * @file		communicator.cpp
 * @brief		Main module for data acquisition and signal output
 * @author		Dr. Klaus Schaefer
 * @copyright 		Copyright 2021 Dr. Klaus Schaefer. All rights reserved.
 * @license 		This project is released under the GNU Public License GPL-3.0

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

#include <FreeRTOS_wrapper.h>
#include "navigator.h"
#include "variometer.h"
#include "NMEA_format.h"
#include "common.h"
#include "organizer.h"
#include "CAN_output.h"
#include "CAN_output_task.h"
#include "D_GNSS_driver.h"
#include "GNSS_driver.h"
#include "CAN_distributor.h"
#include "uSD_handler.h"
#include "uSD_helpers.h"
#include "EEPROM_data_file_implementation.h"
#include "communicator.h"
#include "flexible_log_file_implementation.h"

COMMON D_GNSS_coordinates_t coordinates;
COMMON measurement_data_t observations;
COMMON float3vector external_magnetometer;
COMMON state_vector_t state_vector;

extern "C" void sync_logger (void);

COMMON Semaphore setup_file_handling_completed(1,0,(char *)"SETUP");

COMMON Queue <uint32_t> flight_event_queue(3);
void signal_logger_event( uint32_t event)
{
  flight_event_queue.send( event, 1);
}

COMMON GNSS_type GNSS ( coordinates);

COMMON Queue < communicator_command_t> communicator_command_queue(2);

extern RestrictedTask NMEA_task;
extern RestrictedTask communicator_task;

static ROM bool TRUE=true;
static ROM bool FALSE=false;

typedef struct
{
  float3vector * source;
  float3vector * destination;
  float3vector sum;
  unsigned counter;
} vector_average_organizer_t;

void report_horizon_avalability( void)
{
  if( configuration (HORIZON) )
	update_system_state_clear( HORIZON_NOT_AVAILABLE);
  else
	update_system_state_set( HORIZON_NOT_AVAILABLE);
}

static ROM TaskParameters_t usart_3_task_param =
  {
      USART_3_runnable,
      "USART3",
      256,
      (void *)&TRUE,
      (STANDARD_TASK_PRIORITY+1) | portPRIVILEGE_BIT, // first: start privileged
      0,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { USART_3_RX_buffer, USART_3_RX_BUFFER_SIZE_ROUND_UP, portMPU_REGION_READ_ONLY },
      { 0, 0, 0 }
    }
  };

uint64_t getTime_usec(void);

void
communicator_runnable (void*)
{
  bool have_first_GNSS_fix = false;
  bool fine_tune_sensor_attitude = false;

  // wait until configuration file read if one is given
  setup_file_handling_completed.wait ();

  int64_t time = getTime_usec();

  report_horizon_avalability ();

  vector_average_organizer_t vector_average_organizer = { 0 };
  vector_average_collection_t vector_average_collection = { 0 };

  organizer_t organizer;
  organizer.initialize_before_measurement ();

  GNSS_configration_t GNSS_configuration = (GNSS_configration_t) round (
      configuration (GNSS_CONFIGURATION));
  organizer.set_GNSS_type (GNSS_configuration); // required for speed accuracy monitoring limit value

  switch (GNSS_configuration)
    {
    case GNSS_M9N:
      {
	TaskParameters_t parameters = usart_3_task_param;
	parameters.pvParameters = (void*) &FALSE;

	acquire_privileges ();
	RestrictedTask t (parameters);
	drop_privileges();
      }
      break;
    case GNSS_F9P_F9H: // extra task for 2nd GNSS module required
      {
	  {
	    TaskParameters_t parameters = usart_3_task_param;
	    parameters.pvParameters = (void*) &FALSE;

	    acquire_privileges ();
	    RestrictedTask t (parameters);
	    drop_privileges();

	    Task usart4_task (USART_4_runnable, "D-GNSS", 256, 0,
			      STANDARD_TASK_PRIORITY + 1);
	  }
      }
      break;
    case GNSS_F9P_F9P: // no extra task for 2nd GNSS module
      {
	acquire_privileges ();
	RestrictedTask t (usart_3_task_param);
	drop_privileges();
      }
      break;
    default:
      break;
    }

  for (int i = 0; i < 100; ++i) // wait 1 s until measurement stable
    notify_take (true);

  GNSS.clear_sat_fix_type ();
  GNSS_new_data_ready = false;

  time = getTime_usec() - time;

  // the construction-process may be very slow and shall not wake the watchdog
  // now we can switch to our original priority
  communicator_task.set_priority ( COMMUNICATOR_PRIORITY); // lift priority

  organizer.initialize_after_first_measurement (coordinates, observations);

  NMEA_task.resume ();
  CAN_task.resume ();

  unsigned synchronizer_10Hz = 10; 	// re-sampling 100Hz -> 10Hz
  unsigned GNSS_watchdog = 0;		// monitor incoming GNSS data rate
  unsigned GNSS_LED_count = 0;		// maintain GNSS LED
  unsigned old_system_state = 0; 	// trigger on system state changes
  bool configuration_data_written = false;  // trigger writing of EEPROM content

  // this is the MAIN data acquisition and processing loop **********************************************
  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

      if (GNSS_new_data_ready) // triggered after 75ms or 100ms, GNSS-dependent
	{
	  update_system_state_set (GNSS_AVAILABLE);

	  organizer.update_GNSS_data (coordinates);

	  if (GNSS_configuration > GNSS_M9N)
	    update_system_state_set (D_GNSS_AVAILABLE);

	  if ((have_first_GNSS_fix == false)
	      && (coordinates.sat_fix_type != SAT_FIX_NONE))
	    {
	      have_first_GNSS_fix = true;
	      organizer.update_magnetic_induction_data (coordinates.latitude,
							coordinates.longitude);
	    }

	  GNSS_watchdog = 0;
	}
      else
	{
	  if (GNSS_watchdog < 20)
	    ++GNSS_watchdog;
	  else // we got no data form GNSS receiver
	    {
	      coordinates.sat_fix_type = SAT_FIX_NONE;
	      update_system_state_clear (GNSS_AVAILABLE | D_GNSS_AVAILABLE);
	    }
	}

      organizer.on_new_pressure_data (observations.static_pressure,
				      observations.pitot_pressure);
      organizer.update_at_100_Hz (observations, system_state, external_magnetometer);

      // service external commands if any ***************************************************************
      communicator_command_t command;
      if (communicator_command_queue.receive (command, 0))
	{
	  signal_logger_event( CAN_COMMAND_RECEIVED | (command << 8));

	  switch (command)
	    {
	    case MEASURE_CALIB_LEFT:
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_left);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      break;

	    case MEASURE_CALIB_RIGHT:
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_right);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      break;

	    case MEASURE_CALIB_LEVEL:
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      break;

	    case SET_SENSOR_ROTATION:

	      // make sure that we have all three measurements
	      if (vector_average_collection.acc_observed_left.abs () < 0.001f)
		break;
	      if (vector_average_collection.acc_observed_right.abs () < 0.001f)
		break;
	      if (vector_average_collection.acc_observed_level.abs () < 0.001f)
		break;

	      organizer.update_sensor_orientation_data (
		  vector_average_collection);
	      organizer.initialize_before_measurement ();
	      organizer.initialize_after_first_measurement (coordinates, observations);
	      report_horizon_avalability ();
	      configuration_data_written = false;
	      break;

	    case FINE_TUNE_CALIB: // names "straight flight" in Larus Display Menu
	      vector_average_organizer.source = &(observations.acc);
	      vector_average_organizer.destination =
		  &(vector_average_collection.acc_observed_level);
	      vector_average_organizer.destination->zero ();
	      vector_average_organizer.counter = VECTOR_AVERAGE_COUNT_SETUP;
	      fine_tune_sensor_attitude = true;
	      break;

	    case TIME_CONSTANT_CHANGED:
	    case GNSS_CONFIG_CHANGED:
	      organizer.tune_filters();
	      configuration_data_written = false;
	      break;

	    case TUNE_PRESSURE_GAUGES:
	      organizer.tune_pressure_gauges();
	      configuration_data_written = false;
	      break;

	    case NO_COMMAND:
	      break;
	    }
	}

      // vector averaging in case of ground or air level calibration activity *********************************
      if (vector_average_organizer.counter != 0)
	{
	  vector_average_organizer.sum += *(vector_average_organizer.source);
	  --vector_average_organizer.counter;

	  // if measurement complete now
	  if (vector_average_organizer.counter == 0)
	    {
	      float inverse_count = 1.0f / VECTOR_AVERAGE_COUNT_SETUP;
	      *(vector_average_organizer.destination) =
		  vector_average_organizer.sum * inverse_count;

	      // in this case we do not wait for another command but re-calculate immediately
	      if (fine_tune_sensor_attitude)
		{
		  fine_tune_sensor_attitude = false;
		  organizer.fine_tune_sensor_orientation (
		      vector_average_collection);
		  organizer.initialize_before_measurement ();
		  organizer.initialize_after_first_measurement (coordinates, observations);
		  report_horizon_avalability ();
		  configuration_data_written = false;
		}
	    }
	}

      // slow 10Hz update and landing detection *********************************************************
      --synchronizer_10Hz;
      if (synchronizer_10Hz == 0)
	{
	  synchronizer_10Hz = 10;

	  bool landing_detected_here = organizer.update_at_10Hz (coordinates, observations);
	  if (landing_detected_here)
	    {
	      organizer.cleanup_after_landing ();
	      perform_after_landing_actions.set ();

	      configuration_data_written = false; // needs to be done at next file head
	    }

	  trigger_CAN ();
	}

      // service the GNSS LED ****************************************************************************
      ++GNSS_LED_count;
      GNSS_LED_count &= 0xff;

      switch (GNSS_configuration)
	{
	case GNSS_F9P_F9H:
	case GNSS_F9P_F9P:
	  switch (coordinates.sat_fix_type)
	    {
	    case SAT_FIX:
	      HAL_GPIO_WritePin (
		  LED_STATUS1_GPIO_Port,
		  LED_STATUS1_Pin,
		  ((GNSS_LED_count & 0xe0) == 0xe0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	      break;
	    case SAT_HEADING | SAT_FIX:
	      HAL_GPIO_WritePin (
		  LED_STATUS1_GPIO_Port,
		  LED_STATUS1_Pin,
		  ((GNSS_LED_count & 0x80) && (GNSS_LED_count & 0x20)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	      break;
	    default:
	      HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	      break;
	    }
	  break;
	case GNSS_M9N:
	  if (coordinates.sat_fix_type == SAT_FIX)
	    HAL_GPIO_WritePin (
		LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
		((GNSS_LED_count & 0xe0) == 0xe0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	  else
	    HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	  break;
	default:
	  ASSERT(false);
	  break;
	}

      // service the red error LED ********************************************************************
      HAL_GPIO_WritePin (
	  LED_ERROR_GPIO_Port,
	  LED_ERROR_Pin,
	  essential_sensors_available (GNSS_configuration > GNSS_M9N) ? GPIO_PIN_RESET : GPIO_PIN_SET);

      organizer.report_data (state_vector);

      // write log file ********************************************************************************
      if( flex_file.is_open ()) // data logging is active
	{

	  if( not configuration_data_written) // need to write the EEPROM content
	      {
	      // write all valid EEPROM content packed as one flexible file record
		  {
		    uint32_t file_format_version =
			flexible_log_file_implementation_t::FLEXIBLE_LOG_FILE_FORMAT_VERSION;
		    flex_file.append_record ( FILE_FORMAT_VERSION, &file_format_version, 1);

		    // write hardware, firmware id and FLASH SHA256
#define DESCRIPTION_SIZE_BYTES (32+sizeof( uint32_t)*4+32)
		    uint8_t description[DESCRIPTION_SIZE_BYTES];
		    memset( description, 0, 32);
		    strcpy( (char *)description, GIT_TAG_INFO); // fw string
		    extern  uint32_t UNIQUE_ID[4];
		    memcpy( description+32, UNIQUE_ID, sizeof( uint32_t)*4); // hw ID
		    memcpy( description+32+16, firmware_SHA256_digest, 32); // flash program SHA256 digest
		    flex_file.append_record ( LARUS_DESCRIPTION, (uint32_t *)description, DESCRIPTION_SIZE_BYTES / sizeof( uint32_t));
		  }

		  // write all valid EEPROM content packed as one flexible file record
		  {
#define FLASH_DATA_COPY_SIZE_WORDS 128
		    uint32_t flash_data_copy[FLASH_DATA_COPY_SIZE_WORDS];
		    EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> flash_data_file(
			(EEPROM_file_system_node *)flash_data_copy, (EEPROM_file_system_node *)flash_data_copy+FLASH_DATA_COPY_SIZE_WORDS);
		    flash_data_file.import_all_data( permanent_data_file, false);
		    flex_file.append_record ( EEPROM_FILE, flash_data_copy, flash_data_file.get_size()/sizeof(uint32_t));
		  }

		  configuration_data_written = true;

		  flex_file.append_record (SENSOR_STATUS, &system_state, 1);
		  old_system_state = system_state;
	      }
	  else
	    {
	      if (system_state != old_system_state)
		{
		  flex_file.append_record (SENSOR_STATUS, &system_state, 1);
		  old_system_state = system_state;
		}
	    }

	  flex_file.append_record ( BASIC_SENSOR_DATA, (uint32_t*) &observations, sizeof(observations) / sizeof(uint32_t));

	  if (system_state & EXTERNAL_MAGNETOMETER_AVAILABLE)
	    {
	      flex_file.append_record (
		  MAGNETOMETER_DATA, (uint32_t*) &external_magnetometer,
		  sizeof(external_magnetometer) / sizeof(uint32_t));
	    }

	  if (GNSS_new_data_ready)
	    {
	      GNSS_new_data_ready = false;

	      switch (coordinates.sat_fix_type)
		{
		case SAT_FIX:
		default:
		  flex_file.append_record (
		      GNSS_DATA, (uint32_t*) &coordinates,  sizeof(GNSS_coordinates_t) / sizeof(uint32_t));
		  break;
		case SAT_FIX | SAT_HEADING:
		  flex_file.append_record (
		      D_GNSS_DATA, (uint32_t*) &coordinates, sizeof(D_GNSS_coordinates_t) / sizeof(uint32_t));
		  break;
		case SAT_FIX_NONE: // need to log the GNSS status like number of visible satellites etc
		  flex_file.append_record (
		      GNSS_DATA, (uint32_t*) &coordinates,   sizeof(GNSS_coordinates_t) / sizeof(uint32_t));
		  break;
		}
	    }

	  { // process event if any
	    uint32_t event;
	    if( flight_event_queue.receive( event, 0))
	      flex_file.append_record ( FLIGHT_EVENT, &event, 1);
	  }

	} // log file write loop ****************************************************************************
    }     // IMU 100Hz loop
}         // task runnable

#define STACKSIZE 2048
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  { communicator_runnable, "COM",
  STACKSIZE, 0,
  STANDARD_TASK_PRIORITY, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { (void *)0x080C0000, 0x00040000, portMPU_REGION_READ_WRITE}, // EEPROM
      { &temporary_mag_calculation_data, 8192, portMPU_REGION_READ_WRITE}
    }
  };

COMMON RestrictedTask communicator_task (p);

void
sync_communicator (void) // global synchronization service function
{
  communicator_task.notify_give ();
}
