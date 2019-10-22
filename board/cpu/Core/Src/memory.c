#include "memory.h"

SPI_HandleTypeDef* mem_port = &hspi1;

//-----------------------------------------------------------------------------
// Common constants
//-----------------------------------------------------------------------------
static const uint16_t PAGE_SIZE = 256;

//-----------------------------------------------------------------------------
// Memory operation commands
//-----------------------------------------------------------------------------
static const uint8_t WRITE_ENABLE = 0x06;
static const uint8_t PAGE_PROGRAM = 0x02;
static const uint8_t READ_DATA    = 0x03;


uint32_t TrinityWritePage(uint32_t block, uint32_t sector, uint32_t page, uint8_t* data, uint16_t size) {
    uint32_t address  = 0x00000000;
    uint8_t*  command = (uint8_t*)calloc(MEM_COMMAND_SIZE, sizeof(uint8_t));
    HAL_StatusTypeDef res = HAL_OK;
    // Preparing address
    address |= block  << 16;
    address |= sector << 12;
    address |= page   << 8;
    
    //-------------------------------------------------------------------------
    // Write operation section
    //-------------------------------------------------------------------------
    // Enable memory write
    *command = WRITE_ENABLE;
    res = HAL_SPI_Transmit(mem_port, command, 1, 2000); 
    if (res != HAL_OK) return res;
    //-------------------------------------------------------------------------
    // Write page (data length !!!MUST NOT BE GRATER THAN 256 BYTES!!!) 
    //-------------------------------------------------------------------------
    if (size > PAGE_SIZE) return HAL_ERROR;
    // Preparing command 
    memset(command, 0x00, MEM_COMMAND_SIZE);
    *command = PAGE_PROGRAM;
    memcpy(command + 1, (uint8_t*)&address, 4);
    memcpy(command + 5, data, size);
    res = HAL_SPI_Transmit(mem_port, command, size + 5, 2000);

    free(command);
    return (uint32_t)res;
}

uint32_t TrinityReadPage(uint32_t block, uint32_t sector, uint32_t page, uint8_t* buffer, uint16_t size) {
    uint32_t address  = 0x00000000;
    uint8_t*  command = (uint8_t*)calloc(MEM_COMMAND_SIZE, sizeof(uint8_t));
    HAL_StatusTypeDef res = HAL_OK;
    // Preparing address
    address |= block  << 16;
    address |= sector << 12;
    address |= page   << 8;
    
    //-------------------------------------------------------------------------
    // Read operation sequence
    //-------------------------------------------------------------------------
    if (size > PAGE_SIZE) return HAL_ERROR;
    *command = READ_DATA;
    memcpy(command + 1, (uint8_t*)&address, 4);
    res = HAL_SPI_TransmitReceive(mem_port, command, buffer, size + 5, 2000);
    
    free(command);
    return (uint32_t)res;
}







