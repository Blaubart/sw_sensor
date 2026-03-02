#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "common.h"
#include "main.h"

#if RUN_FLASH_WRITE_TESTER

uint64_t getTime_usec(void);

COMMON unsigned write_test_counter;
COMMON Semaphore trigger_flash_fill( 1,0, (char *)"FLASH_FILL");

static void runnable( void *)
{
  bool success;
  delay( 10000);
  uint64_t time;

  while( true)
    {
      trigger_flash_fill.wait();

      for( write_test_counter=0; write_test_counter < 1250; ++write_test_counter)
      {
        time = getTime_usec();
        success = permanent_data_file.store_data ( 0xa5, 2, &time);
        ASSERT( success);

        HAL_GPIO_WritePin ( LED_STATUS1_GPIO_Port, LED_STATUS1_Pin,
    	  ((( write_test_counter / 100) & 1) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

      uint64_t dummy;
      time = getTime_usec();
      success = permanent_data_file.retrieve_data(0xa5, 2, &dummy);
      time = getTime_usec() - time;

//      suspend();
    }
}

#define STACKSIZE 128
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  {
  runnable,
  "WRITE_TEST",
  STACKSIZE,
  0,
  EEPROM_WRITER_PRIORITY,
  stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { (void *)0x080C0000, 0x00040000, portMPU_REGION_READ_WRITE}, // EEPROM
      { 0, 0, 0}
    }
  };

COMMON RestrictedTask file_write_tester (p);

#endif
