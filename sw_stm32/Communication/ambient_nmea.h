#ifndef AMBIENT_NMEA_H_
#define AMBIENT_NMEA_H_

void send_PLARB_to_ESP32(float battery_voltage,
                         float outside_temperature,
                         float relative_humidity);

#endif