/*stm32f3xx_iic_warpper.h*/
#ifndef __STM32F3XX_IIC_WARPPER_H
#define __STM32F3XX_IIC_WARPPER_H
#include "gd32e10xx_hal.h"

/* Export api */
HAL_StatusTypeDef SendCommand(uint16_t DevAddress, uint8_t *pComData, uint16_t Size);
HAL_StatusTypeDef SendRegCommand(uint16_t DevAddress, uint16_t ComRegAddr, uint16_t ComRegAddrSize, uint8_t *pComData, uint16_t Size);
HAL_StatusTypeDef ReceiveCommand(uint16_t DevAddress, uint8_t *pComData, uint16_t Size);
HAL_StatusTypeDef ReceiveRegCommand(uint16_t DevAddress, uint16_t ComRegAddr, uint16_t ComRegAddrSize, uint8_t *pComData, uint16_t Size);
void MX_I2C0_Init(void);

#endif