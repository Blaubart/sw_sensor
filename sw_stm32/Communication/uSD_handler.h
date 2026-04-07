#ifndef USD_HANDLER_H_
#define USD_HANDLER_H_

#include "FreeRTOS_wrapper.h"
#include "embedded_memory.h"
#include "reminder_flag.h"
#include "flexible_log_file_implementation.h"

extern bool logger_is_enabled;
extern bool magnetic_gound_calibration;
extern bool dump_sensor_readings;

extern flexible_log_file_implementation_t flex_file;
extern reminder_flag perform_after_landing_actions;
extern reminder_flag write_configuration_data_now;

#endif /* USD_HANDLER_H_ */
