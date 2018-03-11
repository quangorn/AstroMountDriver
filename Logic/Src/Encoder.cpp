#include <main.h>
#include <gpio.h>
#include <spi.h>
#include "Encoder.h"

#define HISTORY_SIZE 5

uint32_t m_nSPI_Timeout = 100;

volatile uint32_t m_ValuesHistory[HISTORY_SIZE];
volatile int m_nCurrentHistoryPosition = 0;

uint8_t txBuf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //, 0x00, 0x00};
uint8_t rxBuf[sizeof(txBuf)];

//			R 0xD			1 byte
//ID:		1 1 1 0 1 0 0 1 
//			1 0 0 0 0 1 1 1 //7 bytes from 0x0
//			1 0 1 1 1 1 1 1 //7 bytes from 0x7
//			1 1 1 0 0 1 0 0
void EqUpdateEncoderValues() {
	//Update command
	txBuf[0] = 0xFF;
	HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, txBuf, 1, m_nSPI_Timeout);
	HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1); //TODO: постараться убрать sleep
		
//		//Read 7 bytes from address 0x0
//		txBuf[0] = 0x78;
		
	txBuf[0] = 0x6B; //1 0 0 1 0 1 0 0 //4 bytes from 0x1
	//txBuf[0] = 0x68; //1 0 0 1 0 1 1 1 //7 bytes from 0x1
	HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(&hspi2, txBuf, 1, m_nSPI_Timeout);
	status = HAL_SPI_Receive(&hspi2, rxBuf, 4, m_nSPI_Timeout);
	HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_SET);
	//HAL_Delay(1);
	
	int8_t crcFailed = hspi2.ErrorCode & HAL_SPI_ERROR_CRC;
	int nNewHistoryPosition = (m_nCurrentHistoryPosition + 1) % HISTORY_SIZE;
	m_ValuesHistory[nNewHistoryPosition] = *(uint32_t*)(rxBuf + 1);
	m_nCurrentHistoryPosition = nNewHistoryPosition;
		
//		txBuf[0] = 0x2B; //1 1 0 1 0 1 0 0 //4 bytes from 0xC
//		HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_RESET);
//		status = HAL_SPI_TransmitReceive(&hspi2, txBuf, rxBuf, 5, m_nSPI_Timeout);
//		HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
		
//		//Read 7 bytes from address 0x7
//		//txBuf[0] = 0x40;
//		HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_RESET);
//		status = HAL_SPI_TransmitReceive(&hspi2, txBuf2, rxBuf2, sizeof(txBuf2), m_nSPI_Timeout);
//		HAL_GPIO_WritePin(ENC_RA_CS_GPIO_Port, ENC_RA_CS_Pin, GPIO_PIN_SET);
//		HAL_Delay(1);
}

void EqGetEncoderValues(int16_t* pValueX, int16_t* pValueY) {
	uint32_t tmp = m_ValuesHistory[m_nCurrentHistoryPosition];
	uint8_t* pBytes = (uint8_t*)&tmp;
	*pValueX = pBytes[1] + (pBytes[0] << 8);
	*pValueY = pBytes[3] + (pBytes[2] << 8);
}
