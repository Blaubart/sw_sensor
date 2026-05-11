/***********************************************************************//**
 * @file		CAN_output_task.cpp
 * @brief		CAN output manager
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
#include "FreeRTOS_wrapper.h"
#include "CAN_output.h"
#include "communicator.h"
#include "system_state.h"

COMMON Queue <CANpacket> CAN_pipeline( 5);

bool CAN_enqueue( const CANpacket &p, unsigned max_delay)
{
  return CAN_pipeline.send( p, max_delay);
}

void CAN_task_runnable( void *)
{
  suspend();

  unsigned decimator_1_second=10;

  delay(5000); // allow data acquisition setup

  while( true)
    {
      notify_take(); // synchronize with data acquisition

      bool horizon_available = (system_state & HORIZON_NOT_AVAILABLE) == 0;

      CAN_output( observations, coordinates, state_vector, horizon_available);

      --decimator_1_second;
      if( decimator_1_second < 1)
	{
	  decimator_1_second=10;
	  CAN_heartbeat();
	}

      CANpacket p;
      while( CAN_pipeline.receive( p, 0))
	  CAN_send(p, 1);

    }
}

static ROM TaskParameters_t p =
  {
      CAN_task_runnable,
      "CAN",
      256,
      0,
      CAN_PRIORITY,
      0,
    {
      { COMMON_BLOCK, COMMON_SIZE, portMPU_REGION_READ_WRITE },
      { (void *)0x080C0000, 0x00040000, portMPU_REGION_READ_ONLY }, // EEPROM
      { 0, 0, 0 }
    }
  };

COMMON RestrictedTask CAN_task (p);
