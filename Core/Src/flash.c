#include "flash.h"

HAL_StatusTypeDef FLASH_Write(uint32_t address, uint32_t *data, uint32_t size) {
    HAL_StatusTypeDef status = HAL_OK;

    HAL_FLASH_Unlock();

    // Erase the required flash sectors
    FLASH_Erase_Sector(FLASH_SECTOR_7, VOLTAGE_RANGE_3);

    // Write the data to flash memory
    for (uint32_t i = 0; i < size; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]);
        if (status != HAL_OK) {
            break;
        }
        address += 4; // Increment the address by 4 bytes for the next word
    }

    HAL_FLASH_Lock();

    return status;
}

HAL_StatusTypeDef FLASH_Read(uint32_t address, uint32_t *data, uint32_t size) {
    HAL_StatusTypeDef status = HAL_OK;

    // Read data from Flash memory
    for (uint32_t i = 0; i < size; i++) {
        data[i] = *(__IO uint32_t *)(address + i * 4); // Read data from Flash memory
    }

    return status;
}


