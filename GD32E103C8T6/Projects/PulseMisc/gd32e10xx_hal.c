/**
  ******************************************************************************
  * @file    gd32e10xx_hal.c
  * @author  MCD Application Team
  * @brief   HAL module driver.
  *          This is the common part of the HAL initialization
  *
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    [..]
    The common HAL driver contains a set of generic and common APIs that can be
    used by the PPP peripheral drivers and the user to start using the HAL.
    [..]
    The HAL contains two APIs categories:
         (+) HAL Initialization and de-initialization functions
         (+) HAL Control functions

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gd32e10xx_hal.h"

/* Private macro -------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/** @defgroup HAL_Exported_Variables HAL Exported Variables
  * @{
  */
__IO uint32_t uwTick;
uint32_t uwTickPrio; /* Invalid PRIO define in systick*/
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
      /* Configure the SysTick IRQ priority */
  if (TickPriority < (1UL << __NVIC_PRIO_BITS))
  {
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, TickPriority);
    uwTickPrio = TickPriority;
  } else {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief  This function is called to increment  a global variable "uwTick"
  *         used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *         in SysTick ISR.
  * @note This function is declared as __weak to be overwritten in case of other 
  *         implementations  in user file.
  * @retval None
  */
__weak void HAL_IncTick(void)
{
  uwTick += uwTickFreq;
}

/**
  * @brief  Povides a tick value in millisecond.
  * @note   The function is declared as __Weak  to be overwritten  in case of other 
  *         implementations  in user file.
  * @retval tick value
  */
__weak uint32_t HAL_GetTick(void)
{
  return uwTick;  
}

/**
  * @brief This function returns a tick priority.
  * @retval tick priority
  */
uint32_t HAL_GetTickPrio(void)
{
  return uwTickPrio;
}

/**
  * @brief Set new tick Freq.
  * @retval status
  */
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq)
{
  HAL_StatusTypeDef status  = HAL_OK;
  HAL_TickFreqTypeDef prevTickFreq;

  assert_param(IS_TICKFREQ(Freq));

  if (uwTickFreq != Freq)
  {
    /* Back up uwTickFreq frequency */
    prevTickFreq = uwTickFreq;

    /* Update uwTickFreq global variable used by HAL_InitTick() */
    uwTickFreq = Freq;

    /* Apply the new tick Freq */
    status = HAL_InitTick(uwTickPrio);

    if (status != HAL_OK)
    {
      /* Restore previous tick frequency */
      uwTickFreq = prevTickFreq;
    }
  }

  return status;
}

/**
  * @brief Return tick frequency.
  * @retval tick period in Hz
  */
HAL_TickFreqTypeDef HAL_GetTickFreq(void)
{
  return uwTickFreq;
}

/**
  * @brief  This function provides accurate delay (in milliseconds) based 
  *         on variable incremented.
  * @note   In the default implementation , SysTick timer is the source of time base. 
  *         It is used to generate interrupts at regular time intervals where uwTick
  *         is incremented.
  *         The function is declared as __Weak  to be overwritten  in case of other
  *         implementations  in user file.
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
__weak void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;
  
  /* Add freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  
  while((HAL_GetTick() - tickstart) < wait)
  {
  }
}

/**
  * @brief  Suspend Tick increment.
  * @note   In the default implementation , SysTick timer is the source of time base. It is  
  *         used to generate interrupts at regular time intervals. Once HAL_SuspendTick()
  *         is called, the the SysTick interrupt will be disabled and so Tick increment 
  *         is suspended.
  * @note This function is declared as __weak to be overwritten in case of other
  *         implementations  in user file.
  * @retval None
  */
__weak void HAL_SuspendTick(void)

{
  /* Disable SysTick Interrupt */
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
                                                   
}

/**
  * @brief  Resume Tick increment.
  * @note   In the default implementation , SysTick timer is the source of time base. It is  
  *         used to generate interrupts at regular time intervals. Once HAL_ResumeTick()
  *         is called, the the SysTick interrupt will be enabled and so Tick increment 
  *         is resumed.
  *         The function is declared as __Weak  to be overwritten  in case of other
  *         implementations  in user file.
  * @retval None
  */
__weak void HAL_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
  
}
