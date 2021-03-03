#include "PulseCMD.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"

extern QueueHandle_t xPulseCMDQueue;
extern osThreadId DROTaskHandle;
typedef  void (*pFunction)(void); 
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;

// Protol Header + Length feild.
#define PROTOL_HEADER_LENGTH 5

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
//    CDC_Transmit_FS(ResetAckPacket,6);
//    
//    vTaskDelay(5000 / portTICK_RATE_MS); /*delay to allow the GUI receive ACK*/
//
//    NVIC_SystemReset();
//}

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
    
    CDC_Transmit_FS(UIDAckPacket,18);
}

void MCUGetPIDHandler(void)
{
    uint8_t PIDAckPacket[11];
    PIDAckPacket[0] = 0xAA;
    PIDAckPacket[1] = 0xAA;
    PIDAckPacket[2] = 0x55;
    PIDAckPacket[3] = 0x55;
    PIDAckPacket[4] = 0x06;
    PIDAckPacket[5] = MCU_Get_PID_Response_ACK;
	  PIDAckPacket[6] = MCU_Hardware_Version;
    
    for (uint8_t i=0; i<4; i++)
    {
        PIDAckPacket[i+7] = *(uint8_t*)(P_ID_Address + 3 - i);
    }
    
    CDC_Transmit_FS(PIDAckPacket,11);
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
    VerAckPacket[6] = MCU_Hardware_Version;
    VerAckPacket[7] = MCU_Firmware_Version;
    
    CDC_Transmit_FS(VerAckPacket,8);
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
        CDC_Transmit_FS(LedNackPacket,7);         
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
    
        CDC_Transmit_FS(LedNackPacket,6);  
    }   
}

void MCUStartAcquisitionHandler(void)
{
    uint8_t StartAcqPacket[6];
    StartAcqPacket[0] = 0xAA;
    StartAcqPacket[1] = 0xAA;
    StartAcqPacket[2] = 0x55;
    StartAcqPacket[3] = 0x55;
    StartAcqPacket[4] = 0x01;
    StartAcqPacket[5] = MCU_Start_Acquisition_Response_ACK;
    CDC_Transmit_FS(StartAcqPacket,6); 
    vTaskResume( DROTaskHandle );
    //LED1_OFF();
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
    CDC_Transmit_FS(StopAcqPacket,6); 
    vTaskSuspend( DROTaskHandle );
}

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
        CDC_Transmit_FS(EnDfuAcqPacket,6); 
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

void MCUSetMotorHandler(uint8_t MotorParaL, uint8_t MotorParaH){}

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
    CDC_Transmit_FS(ReadyPacket,6); 
}

void StartCMDTask(void const * argument)
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
                    case MCU_Get_Version:  MCUGetVersionHandler(); break;
                    case MCU_Set_Led:  MCUSetLedHandler(xPulseCMDFrame.ucPulseFrameData[6]); break;
                    case MCU_Start_Acquisition: MCUStartAcquisitionHandler(); break;
                    case MCU_Stop_Acquisition: MCUStopAcquisitionHandler(); break;
                    case MCU_Enter_DFU_Mode: MCUEnterDfuModeHandler(xPulseCMDFrame.ucPulseFrameData[6],xPulseCMDFrame.ucPulseFrameData[7]); break; 
                    case MCU_Set_Motor: MCUSetMotorHandler(xPulseCMDFrame.ucPulseFrameData[6],xPulseCMDFrame.ucPulseFrameData[7]); break;
                    case MCU_Lock_Flash: MCULockFlashHandler(); break;
                    case MCU_Unlock_Flash: MCUUnlockFlashHandler(xPulseCMDFrame.ucPulseFrameData[6],xPulseCMDFrame.ucPulseFrameData[7]); break;    
                    default: break;
                }
            }
        }
    }
  /* USER CODE END StartADCTask */
}
