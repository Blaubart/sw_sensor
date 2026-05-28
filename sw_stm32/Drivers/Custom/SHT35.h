#ifndef SHT35_H_
#define SHT35_H_

#include <stdbool.h>

bool SHT35_init(void);

bool SHT35_read(float &temperature, float &humidity);

#endif