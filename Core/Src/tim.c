/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "stdio.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;     //84Mhz/（83+1）=1Mhz    后面可以试试2MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;     //动态修改ARR值，在这里按照1us固定触发，所以不需要动态修改
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
		printf("HAL_TIM_Base_Init失败！\n"); // 若打印此句，说明基础初始化失败
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
		printf("HAL_TIM_ConfigClockSource失败！\n"); // 时钟源配置失败
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;     //主从同步，定时器作为从机，被外部信号同步，我们定时器作为主机
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
		printf("主从配置失败！\n");
    Error_Handler();
  }


}

void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20;  // 不分频，直接计数TRGO脉冲
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3;  // 最大周期，避免频繁溢出
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;     //主从同步，定时器作为从机，被外部信号同步，我们定时器作为主机
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // TIM3中断优先级：抢占优先级2，子优先级0（低于ADC+DMA）
   HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
}
// 1. 定时器初始化（TIM4，100ms中断一次）
void MX_TIM4_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;               // 84MHz主频→1MHz计数（84-1=83）
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;               // 1MHz×100ms=100000计数（100000-1=99999）
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
	  // TIM4中断优先级：抢占优先级3，子优先级0（低于ADC+DMA）
   HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
   HAL_NVIC_EnableIRQ(TIM4_IRQn);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {

    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

  }
	if(tim_baseHandle->Instance==TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();  // 使能TIM3时钟
  }
	if(tim_baseHandle->Instance==TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();  // 使能TIM4时钟
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {

    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
