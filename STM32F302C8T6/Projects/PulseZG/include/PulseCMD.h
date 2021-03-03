#ifndef __PULSE__CMD__H
#define __PULSE__CMD__H

#include "stdint.h"

/******************************  Bits definition for BME frame ***********************************
  The general frame format 
  ----------------------------------------------------
  |     Byte 1    |  Byte 2  |   Byte 3   |  Byte 4  |
  ----------------------------------------------------
  | Frame control |  Length  | Message ID |  Payload |
  ----------------------------------------------------
  The sensor data frame format 
  ------------------------------------------------------------------
  |     Byte 1    |  Byte 2  |   Byte 3   |    Byte 4    | Byte 5  |
  ------------------------------------------------------------------
  | Frame control |  Length  | Message ID |Sensor Bit Map| Payload |
  ------------------------------------------------------------------
  Lenght    :  The total length of the current data frame.  

  Byte frame header defining the frame type and control flags for MCU and PC communication 
                               
                           Frame control field
  -------------------------------------------------------------------------
  |  Bit7  |  Bit6  |  Bit5  |  Bit4  |  Bit3  |  Bit2  |  Bit1  |  Bit0  |
  -------------------------------------------------------------------------
  |   Frame Type    |  ACK   |  RFU   |  Frame Version  |       Qos       |
  -------------------------------------------------------------------------
  Frame type:  frame packet type
               00 -> command, 01 -> data, 10 -> ack, 11 -> nack.
  ACK:         required the ack packet or not 
               1 -> ack frame required, 0 -> not require ack frame.
  Frame Ver:   communication channel, data pathway.
               00 -> usb, 01 -> nRF24, 10 -> nRF51.
  Qos:         quality of service, packet priority.
               00 -> normal, 01 -> medium, 10-> high, 11 -> very high.
**************************************************************************************************/
/* Bit 7 and 6 in frame control field. */
#define  CMD_TYPE             0x00 /* Command frame */
#define  DATA_TYPE            0x40 /* Data frame */
#define  ACK_TYPE             0x80 /* ACK frame */
#define  NACK_TYPE            0xC0 /* NACK frame */
/* Bit 5 in frame control field. */
#define  ACK_REQ              0x20 /* Ack required */
#define  ACK_NOTREQ           0x00 /* ACK not required */
/* Not used for now. Bit 4 in frame control field, which stands for I/O direction.*/
#define  OUT_FLAG             0x00 /* Flag for output measurement data from MCU to PC */
#define  IN_FLAG              0x10 /* Flag for input command data from PC to MCU */
/* Bit 3 and 2 in frame control field. */
#define  VER_USB              0x00 /* Frame version USB */
#define  VER_nRF24            0x04 /* Frame version nRF24 2.4G wireless */
#define  VER_nRF51            0x08 /* Frame version nRF51 bluetooth or 2.4G */
/* Bit 1 and 0 in frame control field. */
#define  QOS_NORMAL           0x00 /* Data Fragment */
#define  QOS_MEDIUM           0x01 /* N/Ack Fragment*/
#define  QOS_HIGH             0x02 
#define  QOS_VERYHIGH         0x03 

/* NACK error codes */
#define  CmdUnsupported              0x01
#define  ValueOutOfRange             0x02
#define  NotExecutable               0x03
#define  WrongSyntax                 0x04
#define  NotConnected                0x05
#define  CmdDataCorrupt              0x06

//#define  MCU_Reset                               0x22
//#define  MCU_Reset_Response_ACK                  0x24
//#define  MCU_Reset_Response_NACK                 0x28

#define  MCU_Get_PID                             0x22
#define  MCU_Get_PID_Response_ACK                0x24
#define  MCU_Get_PID_Response_NACK               0x28

#define  MCU_Get_ID                              0x32
#define  MCU_Get_ID_Response_ACK                 0x34
#define  MCU_Get_ID_Response_NACK                0x38
#define  MCU_Get_Version                         0x42
#define  MCU_Get_Version_Response_ACK            0x44
#define  MCU_Get_Version_Response_NACK           0x48

#define  MCU_Hardware_Version                    0x02
#define  MCU_Firmware_Version                    0x01

#define  MCU_Set_Led                             0x52
#define  MCU_Set_Led_Response_ACK                0x54
#define  MCU_Set_Led_Response_NACK               0x58
#define  MCU_Start_Acquisition                   0x62
#define  MCU_Start_Acquisition_Response_ACK      0x64
#define  MCU_Start_Acquisition_Response_NACK     0x68
#define  MCU_Stop_Acquisition                    0x72
#define  MCU_Stop_Acquisition_Response_ACK       0x74
#define  MCU_Stop_Acquisition_Response_NACK      0x78
#define  MCU_Enter_DFU_Mode                      0x82
#define  MCU_Enter_DFU_Mode_Response_ACK         0x84
#define  MCU_Enter_DFU_Mode_Response_NACK        0x88
#define  MCU_Set_Motor                           0x92
#define  MCU_Set_Motor_Response_ACK              0x94
#define  MCU_Set_Motor_Response_NACK             0x98

#define  MCU_Send_Ready                          0xA2

#define  MCU_Lock_Flash                          0xB0
#define  MCU_Unlock_Flash                        0xB2

#define  LED1_ON()   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define  LED1_OFF()  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
//#define  LED2_ON()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
//#define  LED2_OFF()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
//#define  LED3_ON()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET)
//#define  LED3_OFF()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET)

#define U_ID_Base_Register_Address   (0x1FFFF7AC)
#define MCU_ID ((const unsigned char *)(U_ID_Base_Register_Address))
#define P_ID_Address                 (0x0800C000)
#define P_ID                         ((const unsigned char *)(P_ID_Address))

#define USBD_DFU_DEFAULT_ADD    0x1FFFD800

typedef struct
{
    uint8_t ucPulseFrameLength;			/*!< The frame length. */
    uint8_t ucPulseFrameData[8];                /*!< The frame data. */
}PulseCMDFrame;

void StartCMDTask(void const * argument);
 
void MCUResetHandler(void);
void MCUGetIDHandler(void);
void MCUGetPIDHandler(void);
void MCUGetVersionHandler(void);

void MCUSetLedHandler(uint8_t LedState);
void MCUStartAcquisitionHandler(void);
void MCUStopAcquisitionHandler(void);
void MCUEnterDfuModeHandler(uint8_t DFUKeyL, uint8_t DFUKeyH);
void MCUSetMotorHandler(uint8_t MotorParaL, uint8_t MotorParaH);

void MCUUnlockFlashHandler(uint8_t ULFKeyL, uint8_t ULFKeyH);
void MCULockFlashHandler(void);

void MCUSendReady(void);

#endif
