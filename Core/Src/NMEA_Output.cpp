#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "common.h"
#include "NMEA_Output.h"

#include <bt_hm.h>
#include "NMEA_format.h"
#include "usb_device.h"
#include "usbd_cdc.h"

COMMON NMEA_buffer_t NMEA_buf;
extern USBD_HandleTypeDef hUsbDeviceFS; // from usb_device.c

static void runnable (void*)
{
#if ACTIVATE_USB_NMEA
  MX_USB_DEVICE_Init();
  update_system_state_set( USB_OUTPUT_ACTIVE);
#endif

  for (synchronous_timer t (NMEA_REPORTING_PERIOD); true; t.sync ())
    {
      char *next;

      format_RMC (GNSS, NMEA_buf.string);
      next = NMEA_append_tail (NMEA_buf.string);

      format_GGA (GNSS, next);  //TODO: ensure that this reports the altitude in meter above medium sea level and height above wgs84: http://aprs.gids.nl/nmea/#gga
      next = NMEA_append_tail (next);

      format_MWV (output_data.wind[NORTH], output_data.wind[EAST], next);
      next = NMEA_append_tail (next);

#if USE_PTAS

      format_PTAS1 (output_data.vario,
		    output_data.integrator_vario,
		    output_data.c.position.e[DOWN] * -1.0,   //TODO: PTAS shall report pure barometric altitude, based on static_pressure. As there can be a QNH applied to in XCSOAR.
		    output_data.TAS,
		    next);
      next = NMEA_append_tail (next);
#endif
      format_POV( output_data.TAS, output_data.m.static_pressure,
			 output_data.m.pitot_pressure, output_data.vario, next);
      next = NMEA_append_tail (next);

      NMEA_buf.length = next - NMEA_buf.string;

#if ACTIVATE_USB_NMEA
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)NMEA_buf.string, NMEA_buf.length);
      USBD_CDC_TransmitPacket(&hUsbDeviceFS);
#endif
#if ACTIVATE_BLUETOOTH_NMEA
      Bluetooth_Transmit( (uint8_t *)(NMEA_buf.string), NMEA_buf.length);
#endif
    }
}

RestrictedTask NMEA_task( runnable, "NMEA", 256, 0, STANDARD_TASK_PRIORITY | portPRIVILEGE_BIT);
