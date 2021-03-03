/*!
    \file  main.c
    \brief led spark and key scan demo
    
    \version 2018-03-26, V1.0.0, demo for GD32E10x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

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

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/
#include <stdio.h>
#include "cmsis_os.h"
/* Using hal tick */
#include "gd32e10xx_hal.h"
/* Using init iic hal */
#include "gd32e10xx_iic_warpper.h"
#include "PulseMisc.h"
/* Using led control */
#include "PulseCMD.h"

#ifdef LOG_DEBUG 
/* Using usart com to debug */
#include "gd32e10x_eval.h"
#endif
/* Private macro -------------------------------------------------------------*/
#define  TICK_INT_PRIORITY            ((uint32_t)15)    /*!< tick interrupt priority (lowest by default)  */   

void init_hal_peripherals(void * pvParameters);

/* Binary semaphore handle definition. */
SemaphoreHandle_t binary_semaphore;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    HAL_InitTick(TICK_INT_PRIORITY);
    
    /* configure 4 bits pre-emption priority */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* init hal interface */
    init_hal_peripherals(NULL);

    /* Start to init real task. */
    CreateWorkerTask();
	
    /* Start scheduler */
    StartWorkerTask();
    
    while (1){
      HAL_Delay(2000);
    }
}

/*!
    \brief      init hal interface
    \param[in]  pvParameters not used
    \param[out] none
    \retval     none
*/
void init_hal_peripherals(void * pvParameters)
{    
#ifdef LOG_DEBUG    
    /* Start init uart0 to debug */
    gd_eval_com_init(EVAL_COM0);
#endif    
    gd_led1_init();
    
    MX_I2C0_Init();
#ifdef LOG_DEBUG    
    printf("\r\nHal interface init successful.\r\n");
#endif    
}
 
#ifdef LOG_DEBUG 
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t) ch);
    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));

    return ch;
}
#endif