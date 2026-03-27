
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "embedded_math.h"

#include "compass_calibrator_3D.h"
#include "magnetic_matrix_calculator.h"
#include "signal_flight_event.h"

#define MAG_CALC_DATA_SIZE 8192
magnetic_calculation_data_t __attribute__((section(".mag_calc_data"))) temporary_mag_calculation_data;
compass_calibrator_3D_t __attribute__((section(".mag_calc_data"))) compass_calibrator_3D( temporary_mag_calculation_data);
compass_calibrator_3D_t __attribute__((section(".mag_calc_data"))) external_compass_calibrator_3D( temporary_mag_calculation_data);

COMMON Queue <bool> calculation_request( 4);

void trigger_compass_calibrator_3D_calculation( bool use_external_magnetometer)
{
  calculation_request.send( use_external_magnetometer);
}

static void magnetic_calculator_runnable ( void *)
{
  bool do_calculate_external_mag;
  while( true)
    {
      calculation_request.receive( do_calculate_external_mag);

      if( do_calculate_external_mag)
	{
	  uint32_t event = EXT_MAG_CALIBRATION_DONE;
	  if( external_compass_calibrator_3D.calculate())
	    event |= 0x100;
	  signal_logger_event( event);
	}
      else
	{
	  uint32_t event = MAG_CALIBRATION_DONE;
	  if( compass_calibrator_3D.calculate())
	    event |= 0x100;
	  signal_logger_event( event);
	}
    }
}

#define STACKSIZE 256
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static TaskParameters_t p =
{
  magnetic_calculator_runnable,
  "MAG_CALC",
  STACKSIZE,
  0,
  MAG_CALCULATOR_PRIORITY,
  stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { &temporary_mag_calculation_data, MAG_CALC_DATA_SIZE, portMPU_REGION_READ_WRITE},
      { 0, 0, 0}
    }
};

static RestrictedTask magnetic_calculator_task (p);

