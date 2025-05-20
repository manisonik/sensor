/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

typedef struct hwtimer
{
  volatile uint32_t sysTickUptime;
  volatile uint32_t sysTickValStamp;
  volatile int sysTickPending;
  uint32_t cpuClockFrequency;
  uint32_t usTicks;
  float usTicksInv;
  /*register*/ volatile uint32_t ms;
  /*register*/ volatile uint32_t cycle_cnt;
  /*register*/ volatile uint32_t pending;
} hwtimer_t;

void initCycleTimer(hwtimer_t* timer);
void timerISR(hwtimer_t* timer);
__attribute__((section(".ram_code"))) __attribute__((noinline)) uint32_t microsISR(hwtimer_t* timer);
uint32_t micros(hwtimer_t* timer);
void delayUs(hwtimer_t* timer, uint32_t us);
void delayMs(hwtimer_t* timer, uint32_t ms);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

