/**
 @file usart_2_driver.h
 @brief usart 2 driver
 @author: Dr. Klaus Schaefer
 */

void USART_2_Init (void);
void USART_2_transmit_DMA( uint8_t *pData, uint16_t Size);
