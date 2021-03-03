/*-------DTSensingDA55M.c-----------*/
#include "stm32f3xx_iic_warpper.h"

// Sensor addr.
#define DTSWriteAddr     0xDA
#define DTSReadAddr      0xDB

/*-----------osr define-------------*/
#define OSR_1024X        0
#define OSR_2048X        1
#define OSR_4096X        2
#define OSR_8192X        3
#define OSR_256X         4
#define OSR_512X         5
#define OSR_16384X       6
#define OSR_32768X       7

/*------------dls define-------------*/
#define OSR_1024X_dls    3//2.5
#define OSR_2048X_dls    4//3.78
#define OSR_4096X_dls    7//6.34
#define OSR_8192X_dls    12//11.46
#define OSR_256X_dls     2//1.54
#define OSR_512X_dls     2//1.86
#define OSR_16384X_dls   22//21.7
#define OSR_32768X_dls   43//42.18

/*-----------read commond define-------*/
#define STATUS_REG       0x02              // Read Status

// Press register.
#define PRESS_REG1      (0x06U)
#define PRESS_REG2      (0x07U)
#define PRESS_REG3      (0x08U)

// Temperature regiester.
#define TEMPERATURE_REG1  (0x09U)
#define TEMPERATURE_REG2  (0x0AU)
#define Sleep_Read       0x0B

// Command register.
#define COMMAND_REG     (0x30U)

/*******************  Bit definition for COMMAND/READ_STATUS register  *******************/
#define DRDY             (0x01U << 0x00U)       // indicates once conversion complete, and the output data is ready for reading.
#define MEASUREMENT_CTRL (0x7U << 0x0U)         // indicate a combined conversion
#define SCO              (0x01U << 0x3U)        // Start of conversion
#define SLEEP_TIME       (0xFFU << 0x4U)        // only active during sleep mode conversion

/**
  // | S | SlaveAddr | 0 | A | StatusReg | A | S | SlaveAddr | 1 | A | Status | A | P |
  // start and send slave address with LSB = 0
  // wait for ACK
  // send status Reg address
  // wait for ACK
  // start and send slave address with LSB = 1
  // receive status data
  // wait for ACK
  // stop
*/
static uint8_t ReadRegStatus( uint8_t StatRegAddr )
{
    uint8_t StatData;

    SendCommand(DTSWriteAddr, &StatRegAddr, 1);
    
    if(HAL_OK != ReceiveCommand(DTSReadAddr, &StatData, 1)) {
        StatData &= ~DRDY;
    }
    
    return StatData;
}

bool ZGReadPressTemp(uint8_t *PreTemData)
{
    uint16_t size = 1;
    uint8_t command;
    // Indicate a combined conversion: P & T.
    uint8_t measurementCtrl = 0x2;
    uint8_t sleepTime = 0U;
    
    do {
      // Send command
      command = sleepTime | SCO | measurementCtrl;
      if(HAL_OK != SendRegCommand(DTSWriteAddr, COMMAND_REG, 1, &command, size))
          break;

      // Delay x ms waiting for conversion complete.
      HAL_Delay(8);

      // read status
      if((ReadRegStatus(STATUS_REG) & DRDY) != true)
          break;

      // read data  
      /* | S | SlaveAddr | 0 | A | PressReg | A | S | SlaveAddr | 1 | A | 
          PressData[2] | A | PressData[1] | A | PressData [0] | A | 
          TempData[1] | A | TempData[0] | N | P | 
      */
        
      command = PRESS_REG1;
      SendCommand(DTSWriteAddr, &command, size);
      size = 5;
      if(HAL_OK != ReceiveCommand(DTSReadAddr, PreTemData, size))
        break;

      return true;
    }while(0);
    
    // Re-initialtion iic controller.
    MX_I2C2_Init();
        
    return false;
}

