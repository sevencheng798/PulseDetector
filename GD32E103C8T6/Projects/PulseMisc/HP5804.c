/*-------HP5804.c-----------*/
#include "gd32e10xx_iic_warpper.h"
#include "gd32e10xx_hal_def.h"
#include "gd32e10xx_hal.h"
#include "PulseMisc.h"

#define HPSWriteAddr     0xEC
#define HPSReadAddr      0xED

/*-----------CMD SET----------------*/
#define CMD_SOFT_RST 0x06 //0000 0110 Soft reset the device
//#define CMD_ADC_CVT  NA   //010_OSR_chnl Perform ADC conversion
#define CMD_READ_PT  0x10 //0001 0000 Read the temperature and pressure values
#define CMD_READ_P   0x30 //0011 0000 Read the pressure value only
#define CMD_READ_A   0x31 //0011 0001 Read the altitude value only
#define CMD_READ_T   0x32 //0011 0010 Read the temperature value only
#define CMD_ANA_CAL  0x28 //0010 1000 Re-calibrate the internal analog blocks
//#define CMD_READ_REG NA   //10_addr   Read out the control registers
//#define CMD_WRITE_REG NA //11_addr Write in the control registers
/*-----------osr define-------------*/
#define OSR_4096X        0
#define OSR_2048X        0x04
#define OSR_1024X        0x08
#define OSR_512X         0x0C
#define OSR_256X         0x10
#define OSR_128X         0x14
#define PREFIX_ADC_CVT   0x40   // The fix prefix '010_'

bool HPReadPressTemp(uint8_t *PreData)
{
    bool result = true; 
    uint16_t size = 1;
    uint8_t command;
    do {
    // First ADC conversion '010_OSR_chnl'.
    command = PREFIX_ADC_CVT|OSR_256X;
    if(HAL_OK != SendCommand(HPSWriteAddr, &command, size))
    {
#if LOG_DEBUG
      printf("\n%s:line:%d : send command [ 0x%x ] failed.\n", __func__, __LINE__, command);
#endif      
      result = false;
      break;
    }
    
    // Delay x ms waiting for conversion complete.
    HAL_Delay(9);
    
    command = CMD_READ_P;
    if(HAL_OK != SendCommand(HPSWriteAddr, &command, size))
    {
#if LOG_DEBUG
      printf("\n%s:line:%d : send command [ 0x%x ] failed.\n", __func__, __LINE__, command);
#endif      
      result = false;
      break;
    }
    
    // The following statement block can be replaced by Hal_Delay().
    for(volatile uint32_t i=0;i<200;i++)
    {
        __NOP();
    }
    
    size = 5;
    if(HAL_OK != ReceiveCommand(HPSReadAddr, PreData, size))
    {
#if LOG_DEBUG
      printf("\n%s:line:%d : receive HP data failed.\n", __func__, __LINE__);
#endif      
      result = false;
      break;
    }

#ifdef GD_SOFT_SIMULATION_IIC
	HAL_Delay(100);
#endif
    } while(0);
    
    // Re-initialtion iic controller.
    if(result != true)
      MX_I2C0_Init();
    
    return result;
}