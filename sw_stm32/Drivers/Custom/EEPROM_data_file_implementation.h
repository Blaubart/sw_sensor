#ifndef CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_
#define CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_

#include "persistent_data_file.h"

typedef struct
{
  uint32_t * dest;
  uint32_t value;
} flash_write_order;

void recover_and_initialize_flash( void);
bool read_blob( EEPROM_file_system_node::ID_t id, unsigned length_in_words, void * data);
bool write_blob( EEPROM_file_system_node::ID_t id, unsigned length_in_words, const void * data);
bool file_system_page_swap( void);

extern EEPROM_file_system <LOWEST_UNUSED_EEPROM_ID> permanent_data_file;

#endif /* CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_ */
