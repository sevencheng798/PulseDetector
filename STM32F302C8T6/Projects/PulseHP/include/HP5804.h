/*HP5804.h*/
#ifndef __HP5804_H
#define __HP5804_H
#include "main.h"

typedef enum {
  false = 0,
  true = !false
} bool;

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

HAL_StatusTypeDef SendRegCommand(uint8_t ComRegAddr, uint8_t ComData);
HAL_StatusTypeDef SendCommand(uint8_t ComData);
uint8_t ReadRegStatus( uint8_t StatRegAddr );
void ReadPressTemp(uint8_t *PreTemData);
bool HPReadPress(uint8_t *PreData);

#endif