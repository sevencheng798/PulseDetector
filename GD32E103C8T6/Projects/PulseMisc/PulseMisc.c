/*!
    \file  PulseMisc.c
    \brief led spark and key scan demo
    
    \version 2020-11-20, V1.0.0, demo for GD32E10x
*/

/*
    Copyright (c) 2018, GM Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.
*/
#include <stdio.h>
/*CM4 API*/
#include "cmsis_os.h"
/* RTOS API */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
/* Usb core hal */
#include "cdc_acm_core.h"
#include "drv_usb_hw.h"
/* Systick hal api */
#include "gd32e10xx_hal.h"
#include "PulseCMD.h"
/* Sensor misc api */
#include "PulseMisc.h"

/* Private define ------------------------------------------------------------*/
#define PROBE_MAX_LOOP 0xFF
/* Definition the stack size requirements for each task. */
#define DEF_DEFAULT_TASK_INSTANCE_MAX           (0U)
#define DEF_DEFAULT_TASK_STACK_SIZE             (64U)
#define DEF_DEFAULT_TASK_OS_PRIORITY            osPriorityAboveNormal //osPriorityAboveNormal
#define DEF_CMD_TASK_INSTANCE_MAX               (0U)
#define DEF_CMD_TASK_STACK_SIZE                 (128U)
#define DEF_CMD_TASK_OS_PRIORITY                osPriorityHigh //osPriorityHigh
#define DEF_DRO_TASK_INSTANCE_MAX               (0U)
#define DEF_DRO_TASK_STACK_SIZE                 (64U)
#define DEF_DRO_TASK_OS_PRIORITY                osPriorityNormal  //osPriorityNormal

/* Private variables ---------------------------------------------------------*/
uint8_t  SendData[64];

osThreadId DefaultTaskHandle;
osThreadId CMDTaskHandle;
osThreadId DROTaskHandle;

/* Exported variables ---------------------------------------------------------*/
/* A RTOS Queue buffer to receive from user command. */
QueueHandle_t xPulseCMDQueue;
/* A handler to point usb core driver. */
usb_core_driver cdc_acm;
/* A flag that sensor type. - defined PulseCMD.c file */
extern uint8_t g_sensorType;

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartCMDTask(void const * argument);
void StartDROTask(void const * argument);
void ProbeSensorType(void);
void init_usb_peripherals(void);

void CreateWorkerTask(void) {
  
#ifdef LOG_DEBUG   
  printf("\r\nStart create work task.\n");
#endif
    
  /* Initialize RTOS Queues buffer*/
  xPulseCMDQueue = xQueueCreate(4, sizeof(PulseCMDFrame));

  /* 
   * Create the thread(s).
   *
   * Definition and creation of DefaultTask 
   */
  osThreadDef(
              DefaultTask,
              StartDefaultTask,
              DEF_DEFAULT_TASK_OS_PRIORITY,
              DEF_DEFAULT_TASK_INSTANCE_MAX,
              DEF_DEFAULT_TASK_STACK_SIZE);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* Definition and creation of CMDTask */
  osThreadDef(
              CMDTask,
              StartCMDTask,
              DEF_CMD_TASK_OS_PRIORITY,
              DEF_CMD_TASK_INSTANCE_MAX,
              DEF_CMD_TASK_STACK_SIZE);
  CMDTaskHandle = osThreadCreate(osThread(CMDTask), NULL);

  /* Definition and creation of DROTask */
  osThreadDef(
              DROTask,
              StartDROTask,
              DEF_DRO_TASK_OS_PRIORITY,
              DEF_DRO_TASK_INSTANCE_MAX,
              DEF_DRO_TASK_STACK_SIZE);
  DROTaskHandle = osThreadCreate(osThread(DROTask), NULL);
  
#ifdef LOG_DEBUG 
  printf("\r\nCreate work task successful.\n");
#endif
}

void StartWorkerTask(void)
{
    /* Start scheduler */
    //osKernelStart();
    vTaskStartScheduler();
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used 
 * @retval None
 */
static void StartDefaultTask(void const * argument)
{
  // First, Suspend the sensor read data task.
  vTaskSuspend( DROTaskHandle );
#ifdef LOG_DEBUG   
   printf("\r\nRunning  DefaultTask successful\r\n");
#endif   
  /**
   * Inital usb hal.
   * Mark: 
   * This USB Hal seems to work only when it is initialized in a thread,
   * so place the block here for gde10x mcu.
   */
  init_usb_peripherals();
  
  // Padding the data packet header.
  SendData[0] = 0xBB; 
  SendData[1] = 0x66;      
  SendData[2] = 0xBB;
  SendData[3] = 0x66;
  
  // Second, Probe sensor type.
  ProbeSensorType();

  // Turn on led1
  LED1_ON();

  // Use the handle to delete the task.
  if( DefaultTaskHandle != NULL )
  {
    vTaskDelete( DefaultTaskHandle );
  }
}

static void StartCMDTask(void const * argument) {
  /**
   * Test command example:
   * 1, get sensor type:
   *  send: 55 55 aa aa 01 a2
   *  recv: AA AA 55 55 02 A4 02 
   * 2, start sensor data:
   *  send: 55 55 aa aa 01 62
   *  recv: AA AA 55 55 01 64 
   *    avild data: BB 66 BB 66 01 00 94 00 00 02 00 00 00 00 
   *               ... ...
   *               ... ...
   * 3, stop sensor data:
   *  send: 55 55 aa aa 01 72
   *  recv: AA AA 55 55 01 74
   */
  // Handler cmd from user by cdc send.
  HandleCMDEvent(NULL);
}

/**
* @brief Function implementing the DROTask thread.
* @param argument: Not used
* @retval None
*/
static void StartDROTask(void const * argument)
{
  /* Infinite loop */
  uint8_t *readData = &SendData[4];
  
  for(;;)
  {

    if(g_sensorType == MCU_Probe_Sensor_HP)
        HPReadPressTemp(readData);  
    else if(g_sensorType == MCU_Probe_Sensor_ZG)
        ZGReadPressTemp(readData);
 
    /* Padding data format
     * | Header | Press data | resverd | SensorType |
     * |  4B    |   3B       |    2B   |    1B      |
     */
    // Pad 2Byte resver and Snr type.
    readData[3] = 0;
    readData[4] = 0;
    readData[5] = g_sensorType;
      
    CDC_Transmit_FS(&cdc_acm, SendData,14);
  }
}

static void ProbeSensorType(void) {
  uint8_t *pRcv = &SendData[4];
  uint32_t *pCheckVaildData = (uint32_t *)pRcv;
  uint8_t loop = 5;
  /**
   * Sensor probe response data format.
   * | Header | Lengths | MsgId | SensorType | TryAgain |
   * | 4Bytes | 1Bytes  | 1Bytes| 1Bytes     |  2Bbytes |
   */
  uint8_t probeData[16]={
          0xaa, 0xaa, 0x55, 0x55, 0x04,
          MCU_Probe_Sensor_Response_NACK,
          0,
          0};
  uint8_t probeDataLengths = 9;
#ifdef LOG_DEBUG   
  printf("\n%s:line:%d : start probe sensor type.\r\n", __func__, __LINE__);
#endif  
  do { 
    // Should to indication a device status turn on.
    LED1_ON();
    
    if(HPReadPressTemp(pRcv)) { // HP Sensor probe.
      if((*pCheckVaildData != 0x00) && 
         (*pCheckVaildData != 0xffffffff)) {
           g_sensorType = MCU_Probe_Sensor_HP;  // HP
#ifdef LOG_DEBUG 
           printf("\r\nread HP data:\r\n");
           for(int i=0; i<16; i++) 
           printf("%x ", SendData[i]);
           printf("\n");
#endif           
           break;
      }
    }
    else if(ZGReadPressTemp(pRcv)) { // ZG Sensor probe.
        if((*pCheckVaildData != 0x00) && 
           (*pCheckVaildData != 0xffffffff)) {
             g_sensorType = MCU_Probe_Sensor_ZG;  // ZG
#ifdef LOG_DEBUG 
           printf("\r\nread ZG data:\r\n");
           for(int i=0; i<16; i++) 
           printf("%x ", SendData[i]);
           printf("\n");
#endif     
             break;
        }
    }
   
    /* Run here will try again. */
#if DEBUG
    loop ++;
    if(loop >= PROBE_MAX_LOOP) {
      probeData[8]++;
      loop = 5;
    }
    
    probeData[7] = loop;
    CDC_Transmit_FS(&cdc_acm, probeData, probeDataLengths);
#endif
    
#ifdef LOG_DEBUG    
    printf("\r\n%s:line:%d : try again to probe sensor type.\r\n", __func__, __LINE__);
#endif
    
    // Should to indication a device status turn off.
    LED1_OFF();
    HAL_Delay(200);
  }while(1);
  
  probeData[5] = MCU_Probe_Sensor_Response_ACK;
  probeData[6] = g_sensorType;
  probeData[7] = loop;
  CDC_Transmit_FS(&cdc_acm, probeData, probeDataLengths);
  
#ifdef LOG_DEBUG 
  printf("\r\nprobe data:\r\n");
  for(int i=0; i<16; i++) 
  printf("%x ", probeData[i]);
  printf("\r\nprobe success.\r\n");
#endif
}
/*!
    \brief      init hal interface
    \param[in]  pvParameters not used
    \param[out] none
    \retval     none
*/
static void init_usb_peripherals(void)
{    
    /* Init USB device Library, add supported class and start the library */
    usb_rcu_config();
    usb_timer_init();
    usbd_init (&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);
    usb_intr_config();

#ifdef USE_IRC48M
    /* CTC peripheral clock enable */
    rcu_periph_clock_enable(RCU_CTC);

    /* CTC configure */
    ctc_config();

    while (ctc_flag_get(CTC_FLAG_CKOK) == RESET) {
    }
#endif
    
#ifdef LOG_DEBUG 
    printf("\r\nUSB Hal interface init successful.\r\n");
#endif
}