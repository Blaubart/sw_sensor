/*
 * spi.cpp
 *
 *  Created on: 24.11.2020
 *      Author: mbetz
 */

#include "spi.h"
#include "main.h"
#include "my_assert.h"
#define SPI_DEFAULT_TIMEOUT_MS  100

void SPI_Init(SPI_HandleTypeDef *hspi)
{

	  hspi1.Instance = SPI1;
	  hspi1.Init.Mode = SPI_MODE_MASTER;
	  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi1.Init.NSS = SPI_NSS_SOFT;
	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi1.Init.CRCPolynomial = 10;
	  if (HAL_SPI_Init(&hspi1) != HAL_OK)
	  {
		  ASSERT(0);
	  }

	  /* SPI2 parameter configuration*/
	  hspi2.Instance = SPI2;
	  hspi2.Init.Mode = SPI_MODE_MASTER;
	  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi2.Init.NSS = SPI_NSS_SOFT;
	  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi2.Init.CRCPolynomial = 10;
	  if (HAL_SPI_Init(&hspi2) != HAL_OK)
	  {
		  ASSERT(0);
	  }
}



void SPI_Transceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, SPI_DEFAULT_TIMEOUT_MS);
	ASSERT(HAL_OK == status);
}


void SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_Transmit(hspi, pTxData, Size, SPI_DEFAULT_TIMEOUT_MS);
	ASSERT(HAL_OK == status);
}

void SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_SPI_Receive(hspi, pRxData, Size, SPI_DEFAULT_TIMEOUT_MS);
	ASSERT(HAL_OK == status);
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
	ASSERT(0);

}
