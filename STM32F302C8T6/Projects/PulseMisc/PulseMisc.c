/*-------PulseMise.c-----------*/
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "PulseCMD.h"
#include "PulseMisc.h"

/* Private define ------------------------------------------------------------*/
#define DEBUG   0
#define PROBE_MAX_LOOP 0xFF
/* Definition the stack size requirements for each task. */
#define DEF_DEFAULT_TASK_INSTANCE_MAX           (0U)
#define DEF_DEFAULT_TASK_STACK_SIZE             (128U)
#define DEF_DEFAULT_TASK_OS_PRIORITY            osPriorityAboveNormal
#define DEF_CMD_TASK_INSTANCE_MAX               (0U)
#define DEF_CMD_TASK_STACK_SIZE                 (128U)
#define DEF_CMD_TASK_OS_PRIORITY                osPriorityHigh
#define DEF_DRO_TASK_INSTANCE_MAX               (0U)
#define DEF_DRO_TASK_STACK_SIZE                 (128U)
#define DEF_DRO_TASK_OS_PRIORITY                osPriorityNormal

extern uint8_t g_sensorType;

/* Private variables ---------------------------------------------------------*/
uint8_t  SendData[64];

osThreadId DefaultTaskHandle;
osThreadId CMDTaskHandle;
osThreadId DROTaskHandle;
// A RTOS Queue buffer to receive from user command.
QueueHandle_t xPulseCMDQueue;

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartCMDTask(void const * argument);
void StartDROTask(void const * argument);
void ProbeSensorType(void);

void CreateWorkerTask(void) {
  /* Initialize RTOS Queues buffer*/
  xPulseCMDQueue = xQueueCreate(4, sizeof(PulseCMDFrame));

  /* 
   * Create the thread(s).
   */
  /* Definition and creation of DefaultTask */
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
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used 
 * @retval None
 */
void StartDefaultTask(void const * argument)
{
  // First, Suspend the sensor read data task.
  vTaskSuspend( DROTaskHandle );
  
  // Padding the data packet header.
  SendData[0] = 0xBB; 
  SendData[1] = 0x66;      
  SendData[2] = 0xBB;
  SendData[3] = 0x66;

#if DEBUG  
  HAL_Delay(5*1000);
#endif
  
  // Second, Probe sensor type.
  ProbeSensorType();

  LED1_ON();

  // Use the handle to delete the task.
  if( DefaultTaskHandle != NULL )
  {
    vTaskDelete( DefaultTaskHandle );
  }
}

void StartCMDTask(void const * argument) {
  HandleCMDEvent(NULL);
}

/**
* @brief Function implementing the DROTask thread.
* @param argument: Not used
* @retval None
*/
void StartDROTask(void const * argument)
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
      
    CDC_Transmit_FS(SendData,14);
  }
}

void ProbeSensorType(void) {
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
  
  do { 
    // Should to indication a device status.
    LED1_ON();
    if(HPReadPressTemp(pRcv)) { // HP Sensor probe.
      if((*pCheckVaildData != 0x00) && 
         (*pCheckVaildData != 0xffffffff)) {
           g_sensorType = MCU_Probe_Sensor_HP;  // HP
           break;
      }
      
    } else if(ZGReadPressTemp(pRcv)) { // ZG Sensor probe.
        if((*pCheckVaildData != 0x00) && 
           (*pCheckVaildData != 0xffffffff)) {
             g_sensorType = MCU_Probe_Sensor_ZG;  // ZG
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
    CDC_Transmit_FS(probeData, probeDataLengths);
#endif
    // Should to indication a device status.
    LED1_OFF();
    HAL_Delay(200);
    
  }while(1);
  
  probeData[5] = MCU_Probe_Sensor_Response_ACK;
  probeData[6] = g_sensorType;
  probeData[7] = loop;
  CDC_Transmit_FS(probeData, probeDataLengths);  
}


