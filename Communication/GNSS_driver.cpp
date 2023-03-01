/**
 @file usart3_driver.cpp
 @brief GNSS USART driver
 @author: Dr. Klaus Schaefer
 */
#include "system_configuration.h"
#include "main.h"
#include "FreeRTOS_wrapper.h"
#include "stm32f4xx_hal.h"
#include "GNSS.h"
#include "GNSS_driver.h"

#if RUN_GNSS

#define DATA_PACKET_TIMEOUT_MS 250 //

COMMON UART_HandleTypeDef huart3;
COMMON DMA_HandleTypeDef hdma_usart3_rx;
COMMON  static TaskHandle_t USART3_task_Id = NULL;

/**
 * @brief USART3 Initialization Function
 */
static inline void MX_USART3_UART_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
      ASSERT(0);

    __HAL_LINKDMA( &huart3, hdmarx, hdma_usart3_rx);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
      ASSERT(0);

    HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, STANDARD_ISR_PRIORITY, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);
}

/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
extern "C" void
DMA1_Stream1_IRQHandler (void)
{
  BaseType_t HigherPriorityTaskWoken=0;
  HAL_DMA_IRQHandler (&hdma_usart3_rx);
  vTaskNotifyGiveFromISR( USART3_task_Id, &HigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(HigherPriorityTaskWoken);
}

#define GPS_DMA_buffer_SIZE (sizeof( uBlox_pvt) + 8) // plus "u B class id size1 size2 ... cks1 cks2"

#define GPS_RELPOS_DMA_buffer_SIZE (sizeof( uBlox_relpos_NED) + 8) // plus "u B class id size1 size2 ... cks1 cks2"
#define RECEIVE_BUFFER_SIZE (GPS_DMA_buffer_SIZE+GPS_RELPOS_DMA_buffer_SIZE)

static uint8_t __ALIGNED(256) buffer[RECEIVE_BUFFER_SIZE];

#if MEASURE_GNSS_REFRESH_TIME
uint64_t getTime_usec_privileged(void);
COMMON uint64_t delta,start,gnss_max;
COMMON uint64_t gnss_min=-1;
#endif

void
USART_3_runnable (void *using_DGNSS)
{
  unsigned buffer_size =
      *(bool*) using_DGNSS ?
	  GPS_DMA_buffer_SIZE + GPS_RELPOS_DMA_buffer_SIZE :
	  GPS_DMA_buffer_SIZE;

  USART3_task_Id = xTaskGetCurrentTaskHandle ();
  MX_USART3_UART_Init ();
  volatile HAL_StatusTypeDef result;

  while (true)
    {

#if MEASURE_GNSS_REFRESH_TIME
      delta = getTime_usec_privileged() - start;
      if( delta >gnss_max)
	gnss_max=delta;
      if( delta < gnss_min)
	gnss_min=delta;
      start = getTime_usec_privileged();
#endif

      result = HAL_UART_Receive_DMA (&huart3, buffer, buffer_size);
      if (result != HAL_OK)
	{
	  HAL_UART_Abort (&huart3);
	  continue;
	}
      // wait for half transfer interrupt
      uint32_t pulNotificationValue;
      BaseType_t notify_result = xTaskNotifyWait(0xffffffff, 0xffffffff,
						 &pulNotificationValue,
						 DATA_PACKET_TIMEOUT_MS);
      if (notify_result != pdTRUE)
	{
	  HAL_UART_Abort (&huart3);
	  continue;
	}
      // wait for transfer complete interrupt
      notify_result = xTaskNotifyWait(0xffffffff, 0xffffffff,
				      &pulNotificationValue,
				      DATA_PACKET_TIMEOUT_MS);
      if (notify_result != pdTRUE)
	{
	  HAL_UART_Abort (&huart3);
	  continue;
	}
      HAL_UART_Abort (&huart3);

      GNSS_Result result;
      if (buffer_size == GPS_DMA_buffer_SIZE + GPS_RELPOS_DMA_buffer_SIZE)
	result = GNSS.update_combined (buffer);
      else
	result = GNSS.update (buffer);

      if (result == GNSS_ERROR)
	{
	  HAL_UART_Abort (&huart3);
	  delay (50);
	}
    }
}

#endif
