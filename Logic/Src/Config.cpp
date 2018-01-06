#include <main.h>
#include <stm32f1xx_hal.h>
#include <string.h>
#include <Protocol.h>
#include "Config.h"

using namespace EQ;

#define FLASH_PAGES_COUNT (*(uint16_t*)FLASH_SIZE_DATA_REGISTER)
#define FLASH_PAGE_ADDR(PageCount) ((void*)(FLASH_BASE + FLASH_PAGE_SIZE * PageCount))
#define FLASH_LAST_PAGE_ADDR FLASH_PAGE_ADDR((FLASH_PAGES_COUNT - 1))

void EqReadConfig(void* ptr) {
//	Config* config = (Config*)ptr;
//	*(uint32_t*)(&config->m_AxisConfigs[MI_RA].m_nMaxFreq) = (uint32_t)FLASH_LAST_PAGE_ADDR;
//	*(uint32_t*)(&config->m_AxisConfigs[MI_DEC].m_nMaxFreq) = 0x800FC00;
//	config->m_AxisConfigs[MI_RA].m_nMaxSpeed = 200;
//	config->m_AxisConfigs[MI_RA].m_nMicrosteps = 200;
	memcpy(ptr, FLASH_LAST_PAGE_ADDR, sizeof(Config));
	//memcpy(ptr, (void*)0x800FC00, sizeof(Config));
}

int EqWriteConfig(void* ptr) {
	uint8_t* begin = (uint8_t*)ptr;
	uint8_t* src = (uint8_t*)ptr;
	uint32_t dst = (uint32_t)FLASH_LAST_PAGE_ADDR;
	uint8_t size = sizeof(Config);
	
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = dst;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError = 0;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
		HAL_FLASH_Lock();
		return STS_FLASH_ERASE_ERROR;
	}
	
	HAL_StatusTypeDef status = HAL_OK;
	while (status == HAL_OK && (src - begin) < size) {
//		if ((src - begin) <= (size - 8)) {
//			status = HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, dst, *(uint64_t*)src);
//			dst += 8;
//			src += 8;
//		} else if ((src - begin) <= (size - 4)) {
//			status = HAL_FLASH_Program(TYPEPROGRAM_WORD, dst, *(uint32_t*)src);
//			dst += 4;
//			src += 4;
//		} else {
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dst, *(uint16_t*)src);
			dst += 2;
			src += 2;
//		}
	}
	
	HAL_FLASH_Lock();   
	return status == HAL_OK ? STS_OK : STS_FLASH_PROGRAM_ERROR;
}
