#include <main.h>
#include <stm32f1xx_hal.h>
#include <string.h>
#include <Protocol.h>
#include "Config.h"

using namespace EQ;

#define FLASH_PAGES_COUNT (*(uint16_t*)FLASH_SIZE_DATA_REGISTER)
#define FLASH_PAGE_ADDR(PageCount) ((void*)(FLASH_BASE + FLASH_PAGE_SIZE * PageCount))
#define FLASH_CONFIG_PAGE FLASH_PAGE_ADDR((FLASH_PAGES_COUNT - 1))
#define FLASH_ENCODER_CORRECTION_PAGE FLASH_PAGE_ADDR((FLASH_PAGES_COUNT - 2))

int ErasePage(void* addr) {
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = (uint32_t)addr;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError = 0;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
		HAL_FLASH_Lock();
		return STS_FLASH_ERASE_ERROR;
	}
	return STS_OK;
};

int WriteData(void* src, void* dst, int size) {
	uint8_t* begin = (uint8_t*)src;
	uint8_t* srcPtr = begin;
	uint32_t dstPtr = (uint32_t)dst;
		
	HAL_FLASH_Unlock();
	
	HAL_StatusTypeDef status = HAL_OK;
	while (status == HAL_OK && (srcPtr - begin) < size) {
//		if ((src - begin) <= (size - 8)) {
//			status = HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, dstPtr, *(uint64_t*)src);
//			dstPtr += 8;
//			srcPtr += 8;
//		} else if ((src - begin) <= (size - 4)) {
//			status = HAL_FLASH_Program(TYPEPROGRAM_WORD, dstPtr, *(uint32_t*)src);
//			dstPtr += 4;
//			srcPtr += 4;
//		} else {
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, dstPtr, *(uint16_t*)srcPtr);
			dstPtr += 2;
			srcPtr += 2;
//		}
	}
	
	HAL_FLASH_Lock();   
	return status == HAL_OK ? STS_OK : STS_FLASH_PROGRAM_ERROR;
}

void EqReadConfig(void* ptr) {
	memcpy(ptr, FLASH_CONFIG_PAGE, sizeof(Config));
}

int EqWriteConfig(void* ptr) {
	int result = ErasePage(FLASH_CONFIG_PAGE);
	if (result != STS_OK) {
		return result;
	}
	
	return WriteData(ptr, FLASH_CONFIG_PAGE, sizeof(Config));
}

void EqReadEncoderCorrection(int pageCount, void* ptr) {
	memcpy(ptr, (uint8_t*)FLASH_ENCODER_CORRECTION_PAGE + pageCount * ENCODER_CORRECTION_PAGE_SIZE, ENCODER_CORRECTION_PAGE_SIZE);
}

int EqWriteEncoderCorrection(int pageCount, void* ptr) {
	return WriteData(ptr, (uint8_t*)FLASH_ENCODER_CORRECTION_PAGE + pageCount * ENCODER_CORRECTION_PAGE_SIZE, ENCODER_CORRECTION_PAGE_SIZE);
}

int EqClearEncoderCorrection() {
	return ErasePage(FLASH_ENCODER_CORRECTION_PAGE);
}
