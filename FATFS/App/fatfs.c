/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"
#include "stdio.h"

char filename[32]="ADC1_DATA.CSV";   /* SD logical drive path *///卷标路径缓冲区，ADC1_DATA不具备意义，初始化的一个东西。


UINT bw;          //实际写入字节数

FATFS fs[_VOLUMES];
FIL *file;
FIL *ftemp;
UINT br, bw;
FILINFO fileinfo;
DIR dir;
uint8_t *fatbuf;     /* SD卡数据缓存区 */



void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
   FATFS_LinkDriver(&SD_Driver, filename);
	
}



/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
