/*-------DTSensingDA55M.c-----------*/
#include <stdint.h>
#include "HP5804.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef SendCommand(uint8_t ComData)
{
    /* | S | SlaveAddr | 0 | A | ComReg | A | CommandData | A | P | */
    // start and send slave address with LSB = 0
    // wait for ACK
    // send command Reg address
    // wait for ACK
    // send command data
    // wait for ACK
    // stop
    //return HAL_I2C_Mem_Write(&hi2c2, HPSWriteAddr, ComRegAddr, 1, &ComData, 1, 0xFF);
    return HAL_I2C_Master_Transmit(&hi2c2, HPSWriteAddr, &ComData, 1, 0xFF);
}

// No used.
HAL_StatusTypeDef SendRegCommand(uint8_t ComRegAddr, uint8_t ComData)
{
    /* | S | SlaveAddr | 0 | A | ComReg | A | CommandData | A | P | */
    // start and send slave address with LSB = 0
    // wait for ACK
    // send command Reg address
    // wait for ACK
    // send command data
    // wait for ACK
    // stop
    return HAL_I2C_Mem_Write(&hi2c2, HPSWriteAddr, ComRegAddr, 1, &ComData, 1, 0xFF);
    //return HAL_I2C_Master_Transmit(&hi2c2, DTSWriteAddr, &StatRegAddr, 1, 0xFF);
}

bool HPReadPress(uint8_t *PreData)
{
    SendCommand(0x40|OSR_256X);
    HAL_Delay(9);
    SendCommand(CMD_READ_P);
    //HAL_Delay(1);
    for(volatile uint32_t i=0;i<200;i++)
    {
        __NOP();
    }	
    if(HAL_OK !=HAL_I2C_Master_Receive(&hi2c2, HPSReadAddr, PreData, 5, 0xFF ))  
    {
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
      
      return false;
    }
    
    return true;
}

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

// No used.
void ReadPressTemp(uint8_t *PreTemData)
{
    uint8_t PressRegAddr = 0x06;
    //uint8_t PreTemData1[5];
    // send command
    SendRegCommand(0x30,0x0A);
    HAL_Delay(8);
    // read status
    if(ReadRegStatus(0x02)==0x01)
    {
        HAL_I2C_Master_Transmit(&hi2c2, HPSWriteAddr, &PressRegAddr, 1, 0xFF);
        HAL_I2C_Master_Receive(&hi2c2, HPSReadAddr, PreTemData, 5, 0xFF );
    }else
    {
        while(1);
    }
    // read data  
    /* | S | SlaveAddr | 0 | A | PressReg | A | S | SlaveAddr | 1 | A | 
        PressData[2] | A | PressData[1] | A | PressData [0] | A | 
        TempData[1] | A | TempData[0] | N | P | */
    
}

