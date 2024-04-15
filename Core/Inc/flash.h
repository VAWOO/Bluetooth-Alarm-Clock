#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef FLASH_Write(uint32_t address, uint32_t *data, uint32_t size);
HAL_StatusTypeDef FLASH_Read(uint32_t address, uint32_t *data, uint32_t size);

#endif /* __FLASH_H */
