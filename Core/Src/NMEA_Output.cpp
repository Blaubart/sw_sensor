/**
 * @file 	NMEA_Output.cpp
 * @brief 	Task for NMEA output to several serial interfaces
 * @author: 	Klaus Schaefer
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
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "NMEA_format.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usart_1_driver.h"
#include "usart_2_driver.h"
#include "communicator.h"
#include "system_state.h"
#include "sensor_dump.h"

COMMON string_buffer_t NMEA_buf;
extern USBD_HandleTypeDef hUsbDeviceFS; // from usb_device.c

static void runnable (void* data)
{

#if ACTIVATE_USB_NMEA
  MX_USB_DEVICE_Init();
  update_system_state_set( USB_OUTPUT_ACTIVE);
#endif

#if ACTIVATE_USART_2_NMEA
  USART_2_Init ();
  update_system_state_set( USART_2_OUTPUT_ACTIVE);
#endif
#if ACTIVATE_USART_1_NMEA
  USART_1_Init ();
  update_system_state_set( USART_1_OUTPUT_ACTIVE);
#endif

#if ACTIVATE_SENSOR_DUMP // Sensor setup version ********************
  MX_USB_DEVICE_Init(); // force using the USB ACM device
  unsigned i=0;
  for ( synchronous_timer t (10); true; t.sync ())
    {
      decimate_sensor_observations( output_data);
      ++i;
      if( i >=10)
	{
	  i=0;
	  format_sensor_dump( output_data, NMEA_buf);
	  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
	  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	}
    }
#endif // Sensor setup version ***************************************

  for (synchronous_timer t (NMEA_REPORTING_PERIOD); true; t.sync ())
    {

      format_NMEA_string( output_data, NMEA_buf);

#if ACTIVATE_USB_NMEA
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
      USBD_CDC_TransmitPacket(&hUsbDeviceFS);
#endif
#if ACTIVATE_BLUETOOTH_NMEA
      Bluetooth_Transmit( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
#if ACTIVATE_USART_1_NMEA
      USART_1_transmit_DMA( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
#if ACTIVATE_USART_2_NMEA
      USART_2_transmit_DMA( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
    }
}

COMMON RestrictedTask NMEA_task( runnable, "NMEA", 256, 0, (NMEA_USB_PRIORITY) | portPRIVILEGE_BIT);

