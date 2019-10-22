#include "main.h"

#ifndef __MEMORY_H__
#define __MEMORY_H__

extern SPI_HandleTypeDef hspi1;

uint32_t TrinityWritePage(uint32_t block, uint32_t sector, uint32_t page, uint8_t* data, uint16_t size);
uint32_t TrinityReadPage(uint32_t block, uint32_t sector, uint32_t page, uint8_t* buffer, uint8_t start_addr, uint16_t size);



#endif