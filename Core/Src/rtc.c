/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"

RTC_HandleTypeDef hrtc;

RTC_TIME current_time;



//月份数据表											 
uint8_t const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表	  
//平年的月份日期表
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};


//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//year,month,day：公历年月日 
//返回值：星期号																						 
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
{	
	uint16_t temp2;
	uint8_t yearH,yearL;
	
	yearH=year/100;	yearL=year%100; 
	// 如果为21世纪,年份数加100  
	if (yearH>19)yearL+=100;
	// 所过闰年数只算1900年之后的  
	temp2=yearL+yearL/4;
	temp2=temp2%7; 
	temp2=temp2+day+table_week[month-1];
	if (yearL%4==0&&month<3)temp2--;
	return(temp2%7);
}
//判断是否是闰年函数
//月份   1  2  3  4  5  6  7  8  9  10 11 12
//闰年   31 29 31 30 31 30 31 31 30 31 30 31
//非闰年 31 28 31 30 31 30 31 31 30 31 30 31
//year:年份
//返回值:该年份是不是闰年.1,是.0,不是
uint8_t Is_Leap_Year(uint16_t year)
{			  
	if(year%4==0) //必须能被4整除
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;//如果以00结尾,还要能被400整除 	   
			else return 0;   
		}else return 1;   
	}else return 0;	
}	
//得到当前的时间，结果保存在RTC_TIME结构体里面
//返回值:0,成功;其他:错误代码.
uint8_t RTC_Get(void)
{
		RTC_TimeTypeDef sTime;      // HAL库时间结构体（时/分/秒）
    RTC_DateTypeDef sDate;      // HAL库日期结构体（年/月/日/星期）
    
    // 1. 读取时间（必须先读时间，再读日期，否则可能出错）
    //   RTC_FORMAT_BIN：返回二进制格式（而非BCD码），方便计算
    if(HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        return 1;  // 读取失败
    }
    
    // 2. 读取日期
    if(HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return 1;  // 读取失败
    }
    
    // 3. 填充时间结构体（直接从HAL库结构体中获取）
    current_time.hour = sTime.Hours;       // 时
    current_time.min  = sTime.Minutes;     // 分
    current_time.sec  = sTime.Seconds;     // 秒
    
    current_time.w_year = 2000 + sDate.Year;  // 年（RTC的Year是相对于2000年的偏移，如23表示2023年）
    current_time.w_month = sDate.Month;       // 月（1-12）
    current_time.w_date = sDate.Date;         // 日（1-31）
    current_time.week = sDate.WeekDay;        // 星期（1-7，HAL库直接返回）  
	return 0;
}	 



//设置时钟
//把输入的时钟转换为秒钟
//以1970年1月1日为基准
//1970~2099年为合法年份
//返回值:0,成功;其他:错误代码.

//syear,smon,sday,hour,min,sec：年月日时分秒
//返回值：设置结果。0，成功；1，失败。
uint8_t RTC_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
	RTC_TimeTypeDef sTime;  // 时间结构体
  RTC_DateTypeDef sDate;  // 日期结构体

  /* 1. 设置时间（先设时间，再设日期） */
  sTime.Hours = hour;         // 小时（0-23）
  sTime.Minutes = min;        // 分钟（0-59）
  sTime.Seconds = sec;        // 秒（0-59）
  sTime.TimeFormat = RTC_HOURFORMAT_24;  // 24小时制
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;  // 不启用夏令时
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;  // 不存储操作
  // 写入时间（格式：RTC_FORMAT_BIN=二进制，直接传整数）
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();  // 设置失败，进入错误处理
  }

  /* 2. 设置日期 */
  sDate.Year = syear - 2000;   // 年份：2025-2000=25（HAL库存的是相对于2000的偏移）
  sDate.Month = smon;        // 月份（1-12）
  sDate.Date = sday;          // 日期（1-31）
  sDate.WeekDay = RTC_Get_Week(syear, smon, sday);  // 星期（1-7，用之前的计算函数）
  // 写入日期（格式同上）
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
    Error_Handler();  // 设置失败
  }
	
	return 0;	    
}

/* RTC init function */
void MX_RTC_Init(void)
{

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;      //（127+1）*（255+1）=32768，即分频到1s
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	if(HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1)!=0X5050)//是否第一次配置
	{
		RTC_Set(2025,10,14,11,07,0); //设置日期和时间，2025年10月11日，13点07分0秒		 									  
		HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0X5050);//标记已经初始化过了
	 	printf("FIRST TIME\n");
	}
  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
