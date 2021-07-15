/** ***********************************************************************
 * @file		communicator.cpp
 * @brief		talk to sensors and synchronize output
 * @author		Dr. Klaus Schaefer
 **************************************************************************/
#include "system_configuration.h"
#include "main.h"

#include <FreeRTOS_wrapper.h>
#include "navigator.h"
#include "flight_observer.h"
#include "serial_io.h"
#include "NMEA_format.h"
#include "common.h"
#include "CAN_output.h"
#include "usart3_driver.h"
#include "usart4_driver.h"

void
sync_logger (void);

COMMON output_data_t __ALIGNED(1024) output_data =
  { 0 };
COMMON GNSS_type GNSS (output_data.c);

extern RestrictedTask NMEA_task;

void communicator_runnable (void*)
{
  navigator_t navigator;
  float3vector acc, mag, gyro;

  // Task usart4_task (USART_4_runnable, "D-GNSS", 256, 0, STANDARD_TASK_PRIORITY+1);

  unsigned airborne_counter = 0;
  unsigned D_GNSS_count;

  GNSS_configration_t GNSS_configuration = (GNSS_configration_t) ROUND (
      configuration (GNSS_CONFIGURATION));
  uint32_t GNSS_sim = 0; // used to synchronize offline calculation w/o GNSS receiver

  float pitot_offset = configuration (PITOT_OFFSET);
  float pitot_span = configuration (PITOT_SPAN);
  float QNH_offset = configuration (QNH_OFFSET);

  float3matrix sensor_mapping;
    {
      quaternion<float> q;
      q.from_euler (configuration (SENS_TILT_ROLL),
		    configuration (SENS_TILT_NICK),
		    configuration (SENS_TILT_YAW));
      q.get_rotation_matrix (sensor_mapping);
    }

#if RUN_CAN_OUTPUT == 1
  uint8_t count_10Hz = 1; // de-synchronize CAN output by 1 cycle
#endif

#ifndef INFILE // only outside of the offline mode
  for (unsigned i = 0; i < 200; ++i) // wait 200 IMU loops
    notify_take (true);

  switch (GNSS_configuration)
    {
    case GNSS_NONE:
      break;
    case GNSS_M9N:
      {
	GNSS.coordinates.relPosHeading = NAN_F; // be sure not to use that one !

	bool use_D_GNSS = false;
	Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&use_D_GNSS, STANDARD_TASK_PRIORITY+1);

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);
	navigator.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;
      }
      break;
    case GNSS_F9P_F9H: // extra task for 2nd GNSS module required
      {
	  {
	    bool use_D_GNSS = false;
	    Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&use_D_GNSS, STANDARD_TASK_PRIORITY+1);

	    Task usart4_task (USART_4_runnable, "D-GNSS", 256, 0, STANDARD_TASK_PRIORITY + 1);
	  }

	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);
	navigator.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;

	float present_heading = 0.0f;
	if (D_GNSS_new_data_ready)
	  present_heading = output_data.c.relPosHeading;
	navigator.set_attitude (0.0f, 0.0f, present_heading);
      }
      break;
    case GNSS_F9P_F9P: // no extra task for 2nd GNSS module
      {
	{
	  bool use_D_GNSS = true;
	  Task usart3_task (USART_3_runnable, "GNSS", 256, (void *)&use_D_GNSS, STANDARD_TASK_PRIORITY+1);
	}
	while (!GNSS_new_data_ready) // lousy spin lock !
	  delay (100);
	navigator.update_GNSS (GNSS.coordinates);
	GNSS_new_data_ready = false;

	if (D_GNSS_new_data_ready && (output_data.c.relPosHeading != NAN_F))
	  navigator.set_attitude (0.0f, 0.0f, output_data.c.relPosHeading);
	else
	  navigator.set_attitude (0.0f, 0.0f, 0.0f); // todo improve me !
      }
      break;
    default:
      ASSERT(false);
    }
#else
    double old_latitude; // used to trigger on new input
#endif

  navigator.update_pabs (output_data.m.static_pressure);
  navigator.reset_altitude ();

  NMEA_task.resume();

  while (true)
    {
      notify_take (true); // wait for synchronization by IMU @ 100 Hz

#ifdef INFILE // we presently run HIL/SIL
      navigator.update_pabs (output_data.m.static_pressure);
      navigator.update_pitot( output_data.m.pitot_pressure);
#else // apply EEPROM configuration data
      navigator.update_pabs (output_data.m.static_pressure - QNH_offset);
      navigator.update_pitot (
	  (output_data.m.pitot_pressure - pitot_offset) * pitot_span);
#endif

      if (navigator.get_IAS () > 20.0f) // are we flying ?
	{
	  if (airborne_counter < 500)
	    airborne_counter = 500;
	  else if (airborne_counter < 2000)
	    ++airborne_counter;
	}
      else
	{
	  if (airborne_counter > 0)
	    {
	      --airborne_counter;
	      if (airborne_counter == 0) // event: landed
		navigator.handle_magnetic_calibration ();
	    }
	}

#ifdef INFILE // we presently run HIL/SIL
	  if (GNSS.coordinates.latitude != old_latitude) // todo this is a dirty workaround
	    {
	      old_latitude = GNSS.coordinates.latitude;
	      GNSS.fix_type = FIX_3d; // has not been recorded ...
	      navigator.update_GNSS (GNSS.coordinates);
	    }
#else
	  if (GNSS_new_data_ready) // triggered at 10 Hz by GNSS
	    {
	      GNSS_new_data_ready = false;
	      navigator.update_GNSS (GNSS.coordinates);
	    }
#endif
      // rotate sensor coordinates into airframe coordinates
      acc = sensor_mapping * output_data.m.acc;
      mag = sensor_mapping * output_data.m.mag;
      gyro = sensor_mapping * output_data.m.gyro;

      navigator.update_IMU (acc, mag, gyro);

      navigator.report_data (output_data);
#if 0 // locost gyreo test
      HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
	  output_data.m.lowcost_gyro[2] < 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
      if( GNSS.coordinates.relPosHeading != NAN_F)
	{
	  ++D_GNSS_count;
	  HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
	      (D_GNSS_count & 0xff) > 127 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
      else
	HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);

#endif

#if RUN_CAN_OUTPUT == 1
      if (++count_10Hz >= 10)
	{
	  count_10Hz = 0;
	  trigger_CAN ();
	}
#endif

      sync_logger (); // kick logger @ 100 Hz
    }
}

#define STACKSIZE 1024 // in 32bit words
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  { communicator_runnable, "COM",
  STACKSIZE, 0,
  COMMUNICATOR_PRIORITY, stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void*) 0x80f8000, 0x10000, portMPU_REGION_READ_WRITE }, // EEPROM access
	  { 0, 0, 0 } } };

COMMON RestrictedTask communicator_task (p);

void
sync_communicator (void) // global synchronization service function
{
  communicator_task.notify_give ();
}

