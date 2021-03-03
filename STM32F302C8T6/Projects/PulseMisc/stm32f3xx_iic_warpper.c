/*-------stm32f3xx_iic_warpper.c-----------*/
#include "stm32f3xx_iic_warpper.h"

I2C_HandleTypeDef hi2c2;

/**
// | S | SlaveAddr | 0 | A | CommandData | A | P |
// start and send slave address with LSB = 0
// wait for ACK
// send command Reg address
// wait for ACK
// send command data
// wait for ACK
// stop
**/
HAL_StatusTypeDef SendCommand
(
    uint16_t DevAddress,
    uint8_t *pComData,
    uint16_t Size
) {
    return HAL_I2C_Master_Transmit(&hi2c2, DevAddress, pComData, Size, 0xFF);
}

/**
// | S | SlaveAddr | 0 | A | ComReg | A | CommandData | A | P |
// start and send slave address with LSB = 0
// wait for ACK
// send command Reg address
// wait for ACK
// send command data
// wait for ACK
// stop
**/
HAL_StatusTypeDef SendRegCommand
(
    uint16_t DevAddress,
    uint16_t ComRegAddr, 
    uint16_t ComRegAddrSize,
    uint8_t *pComData, 
    uint16_t Size
) {
    return HAL_I2C_Mem_Write(&hi2c2, DevAddress, ComRegAddr, ComRegAddrSize, pComData, Size, 0xFF);
}

/**
// | S | SlaveAddr | 0 | A | CommandData | A | S | SlaveAddr | 1 | A | data[7:0] | A | P |
// start and send slave address with LSB = 0
// wait for ACK
// send command Reg address
// wait for ACK
// send command data
// wait for ACK
// stop
 */
HAL_StatusTypeDef ReceiveCommand
(
    uint16_t DevAddress, 
    uint8_t *pComData,
    uint16_t Size
){
    return HAL_I2C_Master_Receive(&hi2c2, DevAddress, pComData, Size, 0xFF);
}

#if 0
// No used.
uint8_t ReadRegStatus( uint8_t StatRegAddr )
{
    uint8_t StatData[2];
    StatData[1] = 0x02;
    //HAL_StatusTypeDef RetValue;
    /* | S | SlaveAddr | 0 | A | StatusReg | A | S | SlaveAddr | 1 | A | Status | A | P | */
    // start and send slave address with LSB = 0
    // wait for ACK
    // send status Reg address
    // wait for ACK
    // start and send slave address with LSB = 1
    // receive status data
    // wait for ACK
    // stop
    //HAL_I2C_Master_Sequential_Transmit_IT(&hi2c2, DTSWriteAddr, &StatData[1], 1, I2C_FIRST_AND_NEXT_FRAME);
    HAL_I2C_Master_Transmit(&hi2c2, HPSWriteAddr, &StatRegAddr, 1, 0xFF);
    //HAL_I2C_Master_Sequential_Receive_IT(&hi2c2, DTSReadAddr, StatData, 1, I2C_LAST_FRAME);
    //RetValue = HAL_I2C_Master_Receive_IT(&hi2c2, DTSReadAddr, StatData, 1);
    HAL_I2C_Master_Receive(&hi2c2, HPSReadAddr, StatData, 1, 0xFF );
    
    return StatData[0];
  
}
#endif

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10808DD3;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END I2C2_Init 1 */
}

