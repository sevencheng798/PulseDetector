#include <stdio.h>
#include "cmsis_os.h"
#include "cdc_acm_core.h"
#include "gd32e10x_fmc.h"

#include "gd32e10xx_hal_def.h"
#include "PulseCMD.h"

// A flag to Indicate sensor type.
uint8_t g_sensorType = MCU_Probe_Sensor_UNKNOW;

// A handler to point usb core driver.
extern usb_core_driver cdc_acm;
extern QueueHandle_t xPulseCMDQueue;
extern osThreadId DROTaskHandle;
typedef  void (*pFunction)(void); 

/* Led1 Control port */
#define LED1_PIN                         GPIO_PIN_12
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK                    RCU_GPIOB

// Protol Header + Length feild.
#define PROTOL_HEADER_LENGTH 5

/*!
    \brief      configure led1 GPIO
    \param      none
    \retval     none
*/
void gd_led1_init (void)
{
    /* enable the led1 clock */
    rcu_periph_clock_enable(LED1_GPIO_CLK);
    /* configure led GPIO port */ 
    gpio_init(LED1_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED1_PIN);

    gd_led1_sda_write(0);
}

/*!
 * \brief turn on/off led1
 * \param[in] bit: specify the led status.
 *   \arg  1 turn off led1
 *   \arg  0 turn on led1
 */
void gd_led1_sda_write(unsigned char bit)
{
    if (bit == 0)
        gpio_bit_reset(LED1_GPIO_PORT, LED1_PIN);
    else
        gpio_bit_set(LED1_GPIO_PORT, LED1_PIN);
}

//void MCUResetHandler(void)
//{
//    uint8_t ResetAckPacket[6];
//    ResetAckPacket[0] = 0xAA;
//    ResetAckPacket[1] = 0xAA;
//    ResetAckPacket[2] = 0x55;
//    ResetAckPacket[3] = 0x55;
//    ResetAckPacket[4] = 0x01;
//    ResetAckPacket[5] = 0x24;
//    
//    CDC_Transmit_FS(&cdc_acm, ResetAckPacket,6);
//    
//    vTaskDelay(5000 / portTICK_RATE_MS); /*delay to allow the GUI receive ACK*/
//
//    NVIC_SystemReset();
//}

uint8_t GetSensorType(void) {
  if(g_sensorType != MCU_Probe_Sensor_UNKNOW)
    return (g_sensorType==MCU_Probe_Sensor_HP)?MCU_Hardware_HP:MCU_Hardware_ZG;
  else
    return MCU_Probe_Sensor_UNKNOW;
}

void MCUGetSensorType(void) {
    uint8_t snrType;
    uint8_t SNRAckPacket[7];
    SNRAckPacket[0] = 0xAA;
    SNRAckPacket[1] = 0xAA;
    SNRAckPacket[2] = 0x55;
    SNRAckPacket[3] = 0x55;   
    
    snrType = GetSensorType();
    if(MCU_Probe_Sensor_UNKNOW != snrType) {
      SNRAckPacket[5] = MCU_Probe_Sensor_Response_ACK;
    } else {
      SNRAckPacket[4] = 0x02;
      SNRAckPacket[5] = MCU_Probe_Sensor_Response_NACK;
    }
    SNRAckPacket[4] = 0x02;
    SNRAckPacket[6] = snrType;
    
    CDC_Transmit_FS(&cdc_acm, SNRAckPacket, 7);
}

void MCUGetIDHandler(void)
{
    uint8_t UIDAckPacket[18];
    UIDAckPacket[0] = 0xAA;
    UIDAckPacket[1] = 0xAA;
    UIDAckPacket[2] = 0x55;
    UIDAckPacket[3] = 0x55;   
    UIDAckPacket[4] = 0x0D;
    UIDAckPacket[5] = MCU_Get_ID_Response_ACK;
    
    for (uint8_t i=0; i<12; i++)
    {
        UIDAckPacket[i+6] = MCU_ID[12-(i+1)];        
    }
    
    CDC_Transmit_FS(&cdc_acm, UIDAckPacket,18);
}

void MCUGetPIDHandler(void)
{
    uint8_t size = 11;
    uint8_t PIDAckPacket[11];
    PIDAckPacket[0] = 0xAA;
    PIDAckPacket[1] = 0xAA;
    PIDAckPacket[2] = 0x55;
    PIDAckPacket[3] = 0x55;
    if(g_sensorType != MCU_Probe_Sensor_UNKNOW) {
      PIDAckPacket[4] = 0x06;
      PIDAckPacket[5] = MCU_Get_PID_Response_ACK;
      PIDAckPacket[6] = MCU_Hardware_Version;        // GetSensorType();
      
      for (uint8_t i=0; i<4; i++)
      {
          PIDAckPacket[i+7] = *(uint8_t*)(P_ID_Address + 3 - i);
      }
    } else {
      size = 7;
      PIDAckPacket[4] = 0x02;
      PIDAckPacket[5] = MCU_Get_PID_Response_NACK;
      PIDAckPacket[6] = 0x01;  // Indicate a err accour.
    }
    
    CDC_Transmit_FS(&cdc_acm, PIDAckPacket, size);
}

void MCUGetVersionHandler(void)
{
    uint8_t VerAckPacket[8];
    VerAckPacket[0] = 0xAA;
    VerAckPacket[1] = 0xAA;
    VerAckPacket[2] = 0x55;
    VerAckPacket[3] = 0x55;
    VerAckPacket[4] = 0x03;
    VerAckPacket[5] = MCU_Get_Version_Response_ACK;
    VerAckPacket[6] = GetSensorType();
    VerAckPacket[7] = MCU_Firmware_Version;
    
    CDC_Transmit_FS(&cdc_acm, VerAckPacket,8);
}

void MCUSetLedHandler(uint8_t LedState)
{  
    uint8_t LedNackPacket[7];
    LedNackPacket[0] = 0xAA;
    LedNackPacket[1] = 0xAA;
    LedNackPacket[2] = 0x55;
    LedNackPacket[3] = 0x55;

    if(LedState > 7)
    {
        LedNackPacket[4] = 0x02;
        LedNackPacket[5] = MCU_Set_Led_Response_NACK;
        LedNackPacket[6] = ValueOutOfRange;
        //SendNACK
        CDC_Transmit_FS(&cdc_acm, LedNackPacket,7);         
    }
    else
    {

        if((LedState & 0x01)==0x01)
        {
            LED1_ON();
        }else LED1_OFF();
    
        if((LedState & 0x02)==0x02)
        {
            LED1_ON();
        }else LED1_OFF();
    
        if((LedState & 0x04)==0x04)
        {
            LED1_ON();
        }else LED1_OFF();

        LedNackPacket[4] = 0x01;
        LedNackPacket[5] = MCU_Set_Led_Response_ACK;
    
        CDC_Transmit_FS(&cdc_acm, LedNackPacket,6);  
    }   
}

void MCUStartAcquisitionHandler(void)
{
    uint8_t type;
    uint8_t size = 6;
    uint8_t StartAcqPacket[7];
    StartAcqPacket[0] = 0xAA;
    StartAcqPacket[1] = 0xAA;
    StartAcqPacket[2] = 0x55;
    StartAcqPacket[3] = 0x55;
    
    type = GetSensorType();
    if(MCU_Probe_Sensor_UNKNOW != type) {
      StartAcqPacket[4] = 0x01;
      StartAcqPacket[5] = MCU_Start_Acquisition_Response_ACK;
    } else {
      size = 7;
      StartAcqPacket[4] = 0x02;
      StartAcqPacket[5] = MCU_Start_Acquisition_Response_NACK;
      StartAcqPacket[6] = 0x01;  // A err accout.
    }
    
    CDC_Transmit_FS(&cdc_acm, StartAcqPacket, size);
    
    /* Dev should to wakeup the read snr data task*/
    if(MCU_Probe_Sensor_UNKNOW != type)
      vTaskResume( DROTaskHandle );
    LED1_OFF();
}

void MCUStopAcquisitionHandler(void)
{
    uint8_t StopAcqPacket[6];
    StopAcqPacket[0] = 0xAA;
    StopAcqPacket[1] = 0xAA;
    StopAcqPacket[2] = 0x55;
    StopAcqPacket[3] = 0x55;
    StopAcqPacket[4] = 0x01;
    StopAcqPacket[5] = MCU_Stop_Acquisition_Response_ACK;
    CDC_Transmit_FS(&cdc_acm, StopAcqPacket,6); 
    vTaskSuspend( DROTaskHandle );
}
#if 0
void MCUEnterDfuModeHandler(uint8_t DFUKeyL, uint8_t DFUKeyH)
{
    if((DFUKeyL == 0x68)&&(DFUKeyH == 0xAE))
    {
        pFunction JumpToApplication;
        __IO uint32_t JumpAddress;
        GPIO_InitTypeDef GPIO_InitStruct = {0};
  
        uint8_t EnDfuAcqPacket[6];
        EnDfuAcqPacket[0] = 0xAA;
        EnDfuAcqPacket[1] = 0xAA;
        EnDfuAcqPacket[2] = 0x55;
        EnDfuAcqPacket[3] = 0x55;
        EnDfuAcqPacket[4] = 0x01;
        EnDfuAcqPacket[5] = MCU_Enter_DFU_Mode_Response_ACK;
        CDC_Transmit_FS(&cdc_acm, EnDfuAcqPacket,6); 
        HAL_Delay(50);

         
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); 
        HAL_Delay(800);	
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); 
        HAL_Delay(8);
        //__disable_irq();
        /* Jump to user application */
        JumpAddress = *(__IO uint32_t*) (USBD_DFU_DEFAULT_ADD + 4);
        JumpToApplication = (pFunction) JumpAddress;      
        /* Initialize user application's Stack Pointer */
        __set_MSP(*(__IO uint32_t*) USBD_DFU_DEFAULT_ADD);
        JumpToApplication();
    }		
}
#endif
void MCUSetMotorHandler(uint8_t MotorParaL, uint8_t MotorParaH){}

void MCUWriteSN(uint32_t SerialNum) {
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t *ptrd = NULL;
  uint8_t SnAcqPacket[8];
  uint8_t size = 6;
  /* Set Response protol header */
  SnAcqPacket[0] = 0xAA;
  SnAcqPacket[1] = 0xAA;
  SnAcqPacket[2] = 0x55;
  SnAcqPacket[3] = 0x55;
  SnAcqPacket[4] = 0x01;
  SnAcqPacket[5] = MCU_Set_SN_Response_ACK;
#if 1
  /*GDFE10X*/
  do {  
    /* unlock the flash program/erase controller */
    fmc_unlock();
    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    /* erase the flash pages */
    if(FMC_READY != fmc_page_erase(P_ID_Address))
      break;
    
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
    
    /* program flash */
    uint32_t Write_Flash_Data = SerialNum;
    if(FMC_READY != fmc_word_program(P_ID_Address, Write_Flash_Data))
       break;
       
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR );
    /* lock the main FMC after the program operation */
    fmc_lock();
  
    /* check flash whether has been programmed */
    ptrd = (uint32_t*)(P_ID_Address);
    if(Write_Flash_Data != (*ptrd))
      break;
  
    status = HAL_OK;
  }while(0);

#else
  /* STM32F10X */
  FLASH_EraseInitTypeDef  flashSelf;
  flashSelf.TypeErase = FLASH_TYPEERASE_PAGES;
  flashSelf.PageAddress = P_ID_Address;
  flashSelf.NbPages = 1;
  
  __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
  do {
  if(HAL_OK != HAL_FLASH_Unlock())
    break;

  uint32_t PageError = 0;
  if(HAL_OK != HAL_FLASHEx_Erase(&flashSelf, &PageError)) {
    HAL_FLASH_Lock();
    break;
  }
  
  uint32_t Write_Flash_Data = SerialNum;
  if(HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, P_ID_Address, Write_Flash_Data)) {
    HAL_FLASH_Lock();
    break;    
  }
  
  HAL_FLASH_Lock();
  
  status = HAL_OK;
  }while(0);
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
 
#endif
  
  if(HAL_OK != status) {
    /* Padding response protocl */
      SnAcqPacket[4] = 0x02;
      SnAcqPacket[5] = MCU_Set_SN_Response_NACK;
      SnAcqPacket[6] = 0x01;    // indicate a err accour.
      size = 7;
  }
  
  CDC_Transmit_FS(&cdc_acm, SnAcqPacket, size); 
}

#if 0
void MCULockFlashHandler(void)
{
    FLASH_OBProgramInitTypeDef OBInit;
  
    __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
  
    HAL_FLASHEx_OBGetConfig(&OBInit);
    if(OBInit.RDPLevel == OB_RDP_LEVEL_0)
    {
        OBInit.OptionType = OPTIONBYTE_RDP;
        OBInit.RDPLevel = OB_RDP_LEVEL_1;
        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();
        HAL_FLASHEx_OBProgram(&OBInit);
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
    }
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    LED1_ON();
}

void MCUUnlockFlashHandler(uint8_t ULFKeyL, uint8_t ULFKeyH)
{
    if((ULFKeyL == 0xC7)&&(ULFKeyH == 0xA8))
    {
        FLASH_OBProgramInitTypeDef OBInit;
  
        __HAL_FLASH_PREFETCH_BUFFER_DISABLE();
  
        HAL_FLASHEx_OBGetConfig(&OBInit);
        if(OBInit.RDPLevel == OB_RDP_LEVEL_1)
        {
            OBInit.OptionType = OPTIONBYTE_RDP;
            OBInit.RDPLevel = OB_RDP_LEVEL_0;
            HAL_FLASH_Unlock();
            HAL_FLASH_OB_Unlock();
            HAL_FLASHEx_OBProgram(&OBInit);
            HAL_FLASH_OB_Lock();
            HAL_FLASH_Lock();
        }
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
        LED1_ON();
    }
}
#endif
void MCUSendReady(void)
{
    uint8_t ReadyPacket[6];
    ReadyPacket[0] = 0xAA;
    ReadyPacket[1] = 0xAA;
    ReadyPacket[2] = 0x55;
    ReadyPacket[3] = 0x55;
    ReadyPacket[4] = 0x01;
    ReadyPacket[5] = MCU_Send_Ready;
    //MCU send Ready packet
    CDC_Transmit_FS(&cdc_acm, ReadyPacket,6); 
}

void HandleCMDEvent(void const * argument)
{
    /* USER CODE BEGIN StartADCTask */
    PulseCMDFrame xPulseCMDFrame;
    /* Infinite loop */
    for(;;)
    {
        /* Use the message queue to wait for the event "frame received from usb". */
        if( xQueueReceive(xPulseCMDQueue, &xPulseCMDFrame, portMAX_DELAY) == pdTRUE)
        {
            if((xPulseCMDFrame.ucPulseFrameData[0] == 0x55) &&
               (xPulseCMDFrame.ucPulseFrameData[1] == 0x55) &&
               (xPulseCMDFrame.ucPulseFrameData[2] == 0xAA) &&
               (xPulseCMDFrame.ucPulseFrameData[3] == 0xAA) &&
               (xPulseCMDFrame.ucPulseFrameLength == xPulseCMDFrame.ucPulseFrameData[4] + PROTOL_HEADER_LENGTH)) {
                  
                switch(xPulseCMDFrame.ucPulseFrameData[5])
                {
                    //case MCU_Reset: MCUResetHandler(); break;
		    case MCU_Get_PID:  MCUGetPIDHandler(); break;
                    case MCU_Get_ID:  MCUGetIDHandler(); break;
                    case MCU_Get_Sensor_Type: MCUGetSensorType(); break;
                    case MCU_Get_Version:  MCUGetVersionHandler(); break;
                    case MCU_Set_Led:  MCUSetLedHandler(xPulseCMDFrame.ucPulseFrameData[6]); break;
                    case MCU_Start_Acquisition: MCUStartAcquisitionHandler(); break;
                    case MCU_Stop_Acquisition: MCUStopAcquisitionHandler(); break;
#if 0 // no use                    
                    case MCU_Enter_DFU_Mode: MCUEnterDfuModeHandler(xPulseCMDFrame.ucPulseFrameData[6],xPulseCMDFrame.ucPulseFrameData[7]); break; 
                    case MCU_Set_Motor: MCUSetMotorHandler(xPulseCMDFrame.ucPulseFrameData[6],xPulseCMDFrame.ucPulseFrameData[7]); break;
                    case MCU_Lock_Flash: MCULockFlashHandler(); break;
                    case MCU_Unlock_Flash: MCUUnlockFlashHandler(xPulseCMDFrame.ucPulseFrameData[6],xPulseCMDFrame.ucPulseFrameData[7]); break;
#endif
                    case MCU_Set_Serial_Number: MCUWriteSN(*(uint32_t*)&xPulseCMDFrame.ucPulseFrameData[6]); break;
                    default: break;
                }
            }
        }
    }
  /* USER CODE END StartADCTask */
}
