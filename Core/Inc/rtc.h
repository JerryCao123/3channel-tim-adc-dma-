/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "stdio.h"
#include "main.h"

typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;			
	//公历日月年周
	uint16_t w_year;
	uint8_t  w_month;
	uint8_t  w_date;
	uint8_t  week;		
}RTC_TIME;

extern RTC_TIME current_time;	
extern RTC_HandleTypeDef hrtc;

uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day);
uint8_t Is_Leap_Year(uint16_t year);
uint8_t RTC_Get(void);
uint8_t RTC_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);
void MX_RTC_Init(void);



//#ifdef __cplusplus
//}
//#endif

#endif /* __RTC_H__ */

