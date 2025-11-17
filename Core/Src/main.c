/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include <string.h>
/***********配置参数****************************************************/
#define THRESHOLD_A     1000        // 阈值a				
#define THRESHOLD_B     2000        // 阈值b
#define THRESHOLD_C     3000        // 阈值c
#define MAX_INTERVALS   50          // 最大有效区间数（冗余设计）

// 1. 结果缓存双缓冲（解决SD卡阻塞）
#define RESULT_BUF_SIZE 20  // 单个结果缓冲区大小（可存300个区间，应对100ms数据量）
//static uint8_t consecutive_low_count = 0;
//// 定义一个需要连续检测的低电平数量，例如 5
//#define CONSECUTIVE_LOW_THRESHOLD 5

/***********ADC双缓冲配置***********************************************/
#define ADC_BUF_SIZE 1024										//单个缓冲区大小
volatile uint16_t ADC_BUFA[ADC_BUF_SIZE]; //缓冲区A
volatile uint16_t ADC_BUFB[ADC_BUF_SIZE]; //缓冲区B
volatile uint16_t ADC_BUFC[ADC_BUF_SIZE]; //缓冲区C
volatile uint16_t ADC_BUFD[ADC_BUF_SIZE]; //缓冲区D
volatile uint16_t ADC_BUFE[ADC_BUF_SIZE]; //缓冲区E
volatile uint16_t ADC_BUFF[ADC_BUF_SIZE]; //缓冲区F
volatile uint8_t BUFA_flag=0;								//缓冲区A就绪标志
volatile uint8_t BUFB_flag=0;								//缓冲区B就绪标志
volatile uint8_t BUFC_flag=0;								//缓冲区C就绪标志
volatile uint8_t BUFD_flag=0;								//缓冲区D就绪标志
volatile uint8_t BUFE_flag=0;								//缓冲区E就绪标志
volatile uint8_t BUFF_flag=0;								//缓冲区F就绪标志
uint32_t BufA_start_us = 0, BufB_start_us = 0;  // 缓冲区起始时间（us）这里用uint64_t，是因为用us计数，如果用32位，71.6分就会数据溢出
uint32_t BufC_start_us = 0, BufD_start_us = 0;
uint32_t BufE_start_us = 0, BufF_start_us = 0;
volatile uint8_t DMA_CurrentBuf=0;  //当前DMA写入的缓冲区（0=BUFA，1=BUFB）
volatile uint8_t DMA2_CurrentBuf=0;
volatile uint8_t DMA3_CurrentBuf=0;
uint16_t *current_buf = NULL;
uint8_t buf_ready_flag = 0;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

/***********时间基准***************************************************/
volatile uint32_t sys_time_us = 0;

/***********sd卡部分参数***********************************************/
FIL fil_a;
UINT bw_a;
FATFS *fs_ptr;  // 定义文件系统指针（用于f_getfree输出）
uint32_t total;  // 总容量
uint32_t free1;   // 剩余容量
extern FATFS fs[_VOLUMES];  // 添加fs的定义
FRESULT fr;       // 操作结果

uint8_t sd_file_opened = 0;  // 标记文件是否已打开


/************有效区间结果结构体*****************************************/
typedef struct 
{
    uint64_t start_time;  // 区间起始时间（us）
		uint8_t channel;      // 通道号
    uint8_t range;        // 范围：1(a<x<b)、2(b<x<c)、3(x>c)
    uint32_t duration;    // 持续时间（us）
    uint16_t max_val;     // 区间最大值
} IntervalResult;
IntervalResult results[MAX_INTERVALS];  // 结果数组
uint8_t result_cnt = 0;                 // 结果计数
// 全局变量：保存上一个缓冲区的结尾状态
//uint8_t last_in_valid = 0;          // 上一个缓冲区结束时是否在有效区间
//uint32_t last_start_time_us = 0;    // 上一个未完成区间的起始绝对时间（us）
//uint16_t last_current_max = 0;      // 上一个未完成区间的最大值
typedef struct {
    uint8_t in_valid;           // 当前是否在有效区间
    uint32_t start_time_us;     // 未完成区间的起始时间
    uint16_t current_max;       // 未完成区间的最大值
} ADC_Status;
ADC_Status adc_status[3] = {0};  // 3路ADC的状态（索引0=ADC1，1=ADC2，2=ADC3）

#define MAX_DURATION_US 1000000  // 最大持续时间阈值（1秒 = 1e6微秒）

/************结果缓冲结构体*********************************************/
typedef struct 
{
  IntervalResult data[RESULT_BUF_SIZE];  // 区间数据数组
  volatile uint16_t cnt;                          // 当前缓存的区间数
  volatile uint8_t ready;                         // 1=缓冲区满，可写入SD卡
	volatile uint32_t start_us;     // 缓冲区开始缓存的时间戳（sys_time_us）
} ResultBuffer;

// 双缓冲实例：A和B
ResultBuffer res_bufA = {0};
ResultBuffer res_bufB = {0};
ResultBuffer res_bufC = {0};
ResultBuffer res_bufD = {0};
ResultBuffer res_bufE = {0};
ResultBuffer res_bufF = {0};
ResultBuffer *current_res_buf[3] = {&res_bufA,&res_bufC,&res_bufE};  // 当前处理逻辑写入的缓冲区


static uint32_t sd_write_count = 0;
/*********************************************************************/
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);//huart1需要根据你的配置修改
	return (ch);
}
// 从 current_time 中获取时间并格式化为字符串（"YYYY-MM-DD HH:MM:SS"）
void get_rtc_time(char* time_buf) {
    // 1. 调用 RTC_Get() 更新 current_time 为最新时间
    RTC_Get();  // 执行后，current_time 已包含当前时间

    // 2. 从 current_time 结构体中提取各时间分量
    uint16_t year  = current_time.w_year;   // 年份（如2025）
    uint8_t  month = current_time.w_month;  // 月份（1-12）
    uint8_t  day   = current_time.w_date;   // 日期（1-31）
    uint8_t  hour  = current_time.hour;     // 小时（0-23）
    uint8_t  min   = current_time.min;      // 分钟（0-59）
    uint8_t  sec   = current_time.sec;      // 秒（0-59）

    // 3. 格式化为字符串（确保两位数对齐，补0）
    // 格式示例："2025-10-18 15:30:45"（共19个字符 + 结束符\0，总20字节）
    sprintf(time_buf, "%04d-%02d-%02d %02d:%02d:%02d",
            year,    // 年（4位，不足补0，如2025）
            month,   // 月（2位，不足补0，如05→5月）
            day,     // 日（2位，不足补0，如03→3日）
            hour,    // 时（2位，不足补0，如09→9点）
            min,     // 分（2位，不足补0，如08→8分）
            sec);    // 秒（2位，不足补0，如02→2秒）
}

// 模拟函数：采集ADC值（需替换为实际ADC读取函数）
float adc_read(void) {
    // 实际应用中需从ADC硬件读取，此处为示例（随机生成10.0~16.0的值）
    return 10.0 + (float)(rand() % 60) / 10.0;  // 10.0~15.9
}

void SD_Init(void)
{
	uint8_t res = 0;
	
	while(MX_SDIO_SD_Init())
	{							//初始化
		printf("SD Card Error!\r\n");
	}
	printf("检测到SD卡!\r\n");
	//相较原子，缺少生成内存空间
	res=f_mount(&fs[0], "0:", 1);        /* 挂载SD卡 */
	if(res != FR_OK)
	{
		 printf("SD卡挂载失败！错误码：%d\r\n", res);
     while (1);
	}
	res=f_stat("0:/",NULL);								//get 文件状态
	if(res==FR_NO_FILE){									//如果无文件
					f_setlabel((const TCHAR *)"0:Betech");			//“磁盘号：卷标”，卷标最大长度11个字符
					res = f_mkfs("0:", 0, 0, 0, _MAX_SS);			  //格式化SD卡，簇大小自动分配，0，0默认即可，缓冲区大小_MAX_SS
					if(res==FR_OK){	
													printf("SD Format Finish\r\n");
							}
							else{
										printf("SD Format Error 错误码：%d\r\n",res);
									}
			}
			else{
						// 已有文件系统，跳过格式化
						printf("SD卡已存在文件系统，无需格式化\r\n");
					}
		
		res=f_open(&fil_a , "0:/ADC_DATA.csv",FA_OPEN_ALWAYS );			//打开文件，写操作
		if(res==FR_OK){
//					f_write (&fil_a, "电压值为10.8V\r\n", 13, &bw_a);								//写具体数据
					f_close(&fil_a);
					printf("文件写入成功！\r\n");
		  }
		  else{
					printf("文件打开失败！错误码：%d\r\n", res);											
				}
		
		 // 获取SD卡容量
    res = f_getfree("0:", &free1, &fs_ptr);										//getSD空余空间
    if (res == FR_OK) {
        total = (fs_ptr->n_fatent - 2) * fs_ptr->csize / 2;   // 总容量(KB)  FATFS表项数*扇区数，扇区数为512，换算成kb就是/2，-2是因为固定2个不可用
        free1 = free1 * fs_ptr->csize / 2;                    // 剩余容量(KB) 剩余表项数*扇区数
        printf("SD卡总容量：%u MB\r\n", total >> 10);					//  >>10即/1024，换算成MB。
        printf("SD卡剩余容量：%u MB\r\n", free1 >> 10);
    } 
		else {
        printf("获取容量失败！错误码：%d\r\n", res);
    }
}



void SystemClock_Config(void);
void ProcessBuffer(uint16_t *buf, uint32_t buf_start_us,uint8_t channel);
void RecordInterval(uint32_t start_idx, uint32_t end_idx, uint16_t max_val,uint8_t channel);
void WriteToSD(IntervalResult *data, uint16_t cnt);
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();

  MX_ADC1_Init();
	MX_ADC2_Init();
  MX_ADC3_Init();
	
  MX_RTC_Init(); 
  MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
  MX_USART1_UART_Init();
	MX_FATFS_Init();
	
	SD_Init();
	
	DMA_CurrentBuf = 0;  // 初始用BUFA
	DMA2_CurrentBuf = 0;  // 初始用BUFC
	DMA3_CurrentBuf = 0;  // 初始用BUFE

	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_BUFA,ADC_BUF_SIZE);  //不要问我为什么，这个要加在开启定时前，
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC_BUFC,ADC_BUF_SIZE); 
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*)ADC_BUFE,ADC_BUF_SIZE); 


	HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);  // 启动定时器（含中断）
	// 5. 启动TIM4（最后启动，非实时任务不影响初始化）
  HAL_TIM_Base_Start_IT(&htim4);  // 启动TIM4，开始100ms异步写入SD卡

	
  BufA_start_us = sys_time_us;  // 初始化第一个缓冲区起始时间
	current_res_buf[0]->start_us = sys_time_us;
	BufC_start_us = sys_time_us;  // 初始化第一个缓冲区起始时间
	current_res_buf[1]->start_us = sys_time_us;
	BufE_start_us = sys_time_us;  // 初始化第一个缓冲区起始时间
	current_res_buf[2]->start_us = sys_time_us;
  while (1)
  {
//		printf("文件指针：%lu\n", fil_a.fptr);
		if(BUFA_flag){
//										HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
										ProcessBuffer(ADC_BUFA, BufA_start_us,0);     //900us
										BUFA_flag=0;
									}
		if(BUFB_flag){
										ProcessBuffer(ADC_BUFB, BufB_start_us,0);
										BUFB_flag=0;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
									}
		if(BUFC_flag){
										ProcessBuffer(ADC_BUFC, BufC_start_us,1);     //
										BUFC_flag=0;	
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

									}
		if(BUFD_flag){
										ProcessBuffer(ADC_BUFD, BufD_start_us,1);
										BUFD_flag=0;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);

									}
		if(BUFE_flag){
										ProcessBuffer(ADC_BUFE, BufE_start_us,2);     //
										BUFE_flag=0;
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
			
									}
		if(BUFF_flag){
										ProcessBuffer(ADC_BUFF, BufF_start_us,2);
										BUFF_flag=0;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
									}

  }
  
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage**/
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

//缓冲区处理函数
void ProcessBuffer(uint16_t *buf, uint32_t buf_start_us,uint8_t channel)
{
	uint8_t in_valid = adc_status[channel].in_valid;  // 继承上一个缓冲区的状态，是否在有效区间
  uint32_t start_time_us = 0;            // 区间起始绝对时间（而非索引）
  uint16_t current_max =0;			//区间最大值
	
	
	// 若上一个缓冲区有未完成的有效区间，继承其起始时间和最大值
  if (in_valid) 
	{
    start_time_us = adc_status[channel].start_time_us;
    current_max = adc_status[channel].current_max;
  } 

	
	//遍历1024个数据
  for (uint16_t i = 0; i < ADC_BUF_SIZE; i++) 
	{
    uint16_t x = buf[i];
		uint32_t current_time_us = buf_start_us + i;  // 当前数据的绝对时间（us）				
    if (x > THRESHOLD_A)		//是否在有效区间
		{
//			consecutive_low_count = 0; // 检测到高电平，重置低电平计数器
      if (!in_valid)				//如果不在
				{
					in_valid = 1;			//开始有效区间置1
					start_time_us = current_time_us;  // 记录起始绝对时间;		
					current_max = x;	//最大值赋值
				} 
			else {								//如果在
							if (x > current_max) current_max = x;			//判断当前值是否大于记录值，如果大，最大值替换为当前值
				
							 // 核心修改：检查是否超过1秒阈值，超过则强制分割
							if (current_time_us - start_time_us >= MAX_DURATION_US)
							{
								// 记录到当前时间点的区间
								RecordInterval(start_time_us, current_time_us, current_max,channel);
								// 开启新的区间（从下一个数据开始）
								start_time_us = current_time_us + 1;
								current_max = x;  // 新区间初始最大值为当前值
								}
						}
    } 
		else 										//如果不在有效区间
				{
					
						if (in_valid) 		//判断之前是否在有效区间
						{ 
//									consecutive_low_count++;
//									// 只有当连续检测到 CONSECUTIVE_LOW_THRESHOLD 个低电平时，才确认区间结束
//									if (consecutive_low_count >= CONSECUTIVE_LOW_THRESHOLD)
//									{
										in_valid = 0;			//如果之前在，赋值让它不在
										RecordInterval(start_time_us, current_time_us-1, current_max,channel);		//记录结果
//										consecutive_low_count = 0; // 重置计数器
//									}
						}
//						 else
//							{
//										consecutive_low_count = 0;
//							}
				}
  }

  // 处理当前缓冲区末尾的未完成区间（保存到全局变量，供下一个缓冲区衔接）
  adc_status[channel].in_valid = in_valid;
  if (in_valid)
	{
    adc_status[channel].start_time_us = start_time_us;
    adc_status[channel].current_max = current_max;
  }
//	adc1_processed++;
	
  // 强制写入剩余数据（即使未满MAX_INTERVALS）
//  if (result_cnt > 0)								//result_cnt表示计算当前已缓存的有效区间数量，如果长期在有效区间，实际它的值为0，这样就会卡着，一直不写数据进SD卡。
//	{
//    WriteToSD();
//			result_cnt = 0;
//  }
}
// 记录区间信息
void RecordInterval(uint32_t start_us, uint32_t end_us, uint16_t max_val,uint8_t channel) //修改：start_us，end_us原uint16_t改成uint32_t
{
	ResultBuffer *buf = current_res_buf[channel];
	if (buf->cnt == 0) buf->start_us = sys_time_us;
	// 缓冲满则切换
    if (buf->cnt >= RESULT_BUF_SIZE)
    {
        buf->ready = 1;
				if(channel==0){current_res_buf[0]=(current_res_buf[0]==&res_bufA)? &res_bufB : &res_bufA;}
				else if(channel==1){current_res_buf[1]=(current_res_buf[1]== &res_bufC)? &res_bufD : &res_bufC;}
				else if(channel==2){current_res_buf[2]=(current_res_buf[2]==&res_bufE)? &res_bufF : &res_bufE;}
        return;
    }
	// 若当前缓冲区是新切换的（cnt=0），记录开始缓存的时间
//  if (current_res_buf->cnt == 0) {
//    current_res_buf->start_us = sys_time_us;  // 用sys_time_us标记开始时间
//  }
	
//	if (current_res_buf->cnt >= RESULT_BUF_SIZE) 
//	{
//			current_res_buf->ready = 1;  // 标记为可写入SD
//			// 切换缓冲区（A→B或B→A）
//			current_res_buf = (current_res_buf == &res_bufA) ? &res_bufB : &res_bufA;
//		return;  // 切换后直接返回，新缓冲区的start_us将在下次调用时初始化
//  }
	// 新增超时触发逻辑：即使cnt未达标，缓存超过2秒也强制置位ready
  uint32_t current_us = sys_time_us;
  if (current_us - buf->start_us >= 2000000) {  // 2秒=2,000,000us
    buf->ready = 1;  // 超时强制标记为就绪
    if(channel==0){current_res_buf[0]=(current_res_buf[0]== &res_bufA)? &res_bufB : &res_bufA;}
		else if(channel==1){current_res_buf[1]=(current_res_buf[1]== &res_bufC)? &res_bufD : &res_bufC;}
		else if(channel==2){current_res_buf[2]=(current_res_buf[2]== &res_bufE)? &res_bufF : &res_bufE;}
    return;
  }
//  if (result_cnt >= MAX_INTERVALS) 
//	{
//		WriteToSD();  // 临时写入当前数据
//    result_cnt = 0;  // 重置计数器，准备记录新区间
//	}

//  IntervalResult *res = &results[result_cnt];
	IntervalResult *res = &buf->data[buf->cnt];
  res->duration = end_us - start_us + 1;  // 持续时间（us）//+1

  if (max_val < THRESHOLD_B) res->range = 1;
  else if (max_val < THRESHOLD_C) res->range = 2;
  else res->range = 3;
  res->max_val = max_val;
	res->channel = channel + 1;
	buf->cnt++;  // 计数+1
//  result_cnt++;
}
void WriteToSD(IntervalResult *data, uint16_t cnt) 
{
  if (cnt == 0) return;
	
	FILINFO fno;  // 文件信息结构体（用于判断文件是否存在）
  char rtc_time[20];             // 存放RTC时间字符串
  char line[64];
	
	UINT bw;  // 局部变量，避免全局变量干扰
	
  RTC_Get();
	
	uint16_t write_len;

	if (!sd_file_opened) 
		{
			fr = f_open(&fil_a, "0:/ADC_DATA.csv", FA_WRITE | FA_OPEN_APPEND);// | FA_CREATE_NEW
			if (fr != FR_OK) {
        printf("文件打开失败！错误码：%d\r\n", fr);
        return;
    }
			if (fr == FR_EXIST) {  // 若文件已存在，直接打开追加
														fr = f_open(&fil_a, "0:/ADC_DATA.csv", FA_WRITE | FA_OPEN_APPEND);
													}

							// 首次打开文件时，写入标题行（方便表格识别列）
				const char *title = "时间,通道,档位,持续时间(us),最大值(V)\r\n";
				fr = f_write(&fil_a, title, strlen(title), &bw);
													
				sd_file_opened = 1;
		}
	
  for (uint8_t i = 0; i < cnt; i++) 
	{
		// 有效：格式化一行数据（带时间戳和换行）
    get_rtc_time(rtc_time);  // 获取当前时间
    IntervalResult *res = &data[i];
	 // 格式化数据（确保字符串有效）
    write_len = sprintf(line, "%s,%hhu,%hhu,%lu,%.2f\r\n",
                        rtc_time,res->channel, res->range, res->duration, (res->max_val*0.0241));
		fr = f_write(&fil_a, line, strlen(line), &bw);//bw_a
			if (fr != FR_OK) {
//			f_open(&fil_a, "0:/ADC_DATA.csv", FA_WRITE | FA_OPEN_APPEND);
            printf("写入失败！错误码：%d，行：%s\r\n", fr, line);
            break;
        }
  }

	f_sync(&fil_a);
//	__enable_irq;
//  f_close(&fil_a);

  
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) 
{		
if (hadc == &hadc1) {
//	  adc1_total++;  // 每填满一个缓冲区，总计数+1
    // 1. 先停止当前DMA（确保安全切换地址）
    HAL_DMA_Abort(&hdma_adc1);

    // 2. 根据当前缓冲区，切换到另一个缓冲区+更新标志
    if (DMA_CurrentBuf == 0) {  // 上一轮用的是BUFA，这次切换到BUFB
      BUFA_flag = 1;            // 通知主循环处理BUFA
      BufA_start_us = sys_time_us - ADC_BUF_SIZE;  // 补BUFA的起始时间（传输用了ADC_BUF_SIZE个us）
      
      // 配置DMA下一轮传输：内存地址=BUFB，长度=ADC_BUF_SIZE
      hdma_adc1.Instance->M0AR = (uint32_t)ADC_BUFB;
      DMA_CurrentBuf = 1;       // 标记当前缓冲区为BUFB
    } else {  // 上一轮用的是BUFB，这次切换到BUFA
      BUFB_flag = 1;            // 通知主循环处理BUFB
      BufB_start_us = sys_time_us - ADC_BUF_SIZE;  // 补BUFB的起始时间
      
      // 配置DMA下一轮传输：内存地址=BUFA，长度=ADC_BUF_SIZE
      hdma_adc1.Instance->M0AR = (uint32_t)ADC_BUFA;
      DMA_CurrentBuf = 0;       // 标记当前缓冲区为BUFA
    }

    // 3. 重启DMA（关键：普通模式需手动重启，外设地址固定为ADC1->DR）
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)hdma_adc1.Instance->M0AR, ADC_BUF_SIZE);

  
  }
if (hadc == &hadc2) {
    // 1. 先停止当前DMA（确保安全切换地址）
    HAL_DMA_Abort(&hdma_adc2);

    // 2. 根据当前缓冲区，切换到另一个缓冲区+更新标志
    if (DMA2_CurrentBuf == 0) {  // 上一轮用的是BUFA，这次切换到BUFB
      BUFC_flag = 1;            // 通知主循环处理BUFA
      BufC_start_us = sys_time_us - ADC_BUF_SIZE;  // 补BUFA的起始时间（传输用了ADC_BUF_SIZE个us）
      
      // 配置DMA下一轮传输：内存地址=BUFB，长度=ADC_BUF_SIZE
      hdma_adc2.Instance->M0AR = (uint32_t)ADC_BUFD;
      DMA2_CurrentBuf = 1;       // 标记当前缓冲区为BUFB
    } else {  // 上一轮用的是BUFB，这次切换到BUFA
      BUFD_flag = 1;            // 通知主循环处理BUFB
      BufD_start_us = sys_time_us - ADC_BUF_SIZE;  // 补BUFB的起始时间
      
      // 配置DMA下一轮传输：内存地址=BUFA，长度=ADC_BUF_SIZE
      hdma_adc2.Instance->M0AR = (uint32_t)ADC_BUFC;
      DMA2_CurrentBuf = 0;       // 标记当前缓冲区为BUFA
    }

    // 3. 重启DMA（关键：普通模式需手动重启，外设地址固定为ADC1->DR）
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)hdma_adc2.Instance->M0AR, ADC_BUF_SIZE);

  
  }
if (hadc == &hadc3) {
	
    // 1. 先停止当前DMA（确保安全切换地址）
    HAL_DMA_Abort(&hdma_adc3);

    // 2. 根据当前缓冲区，切换到另一个缓冲区+更新标志
    if (DMA3_CurrentBuf == 0) {  // 上一轮用的是BUFA，这次切换到BUFB
      BUFE_flag = 1;            // 通知主循环处理BUFA
      BufE_start_us = sys_time_us - ADC_BUF_SIZE;  // 补BUFA的起始时间（传输用了ADC_BUF_SIZE个us）
      
      // 配置DMA下一轮传输：内存地址=BUFB，长度=ADC_BUF_SIZE
      hdma_adc3.Instance->M0AR = (uint32_t)ADC_BUFF;
      DMA3_CurrentBuf = 1;       // 标记当前缓冲区为BUFB
    } else {  // 上一轮用的是BUFB，这次切换到BUFA
      BUFF_flag = 1;            // 通知主循环处理BUFB
      BufF_start_us = sys_time_us - ADC_BUF_SIZE;  // 补BUFB的起始时间
      
      // 配置DMA下一轮传输：内存地址=BUFA，长度=ADC_BUF_SIZE
      hdma_adc3.Instance->M0AR = (uint32_t)ADC_BUFE;
      DMA3_CurrentBuf = 0;       // 标记当前缓冲区为BUFA
    }

    // 3. 重启DMA（关键：普通模式需手动重启，外设地址固定为ADC1->DR）
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)hdma_adc3.Instance->M0AR, ADC_BUF_SIZE);

  
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
 if (htim->Instance == TIM3)
	{
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
    sys_time_us++;  // 每1us递增，无其他操作
		return;
  }
 if (htim->Instance == TIM4) 
	{  
//		HAL_NVIC_DisableIRQ(TIM3_IRQn);
		
    // 优先处理res_bufA
    if (res_bufA.ready) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			 sd_write_count++;
//      printf("SD写入操作次数: %lu\n", sd_write_count);
      WriteToSD(res_bufA.data, res_bufA.cnt);//31ms
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
      res_bufA.cnt = 0;        // 清空缓冲区
      res_bufA.ready = 0;      // 标记为未就绪
			res_bufA.start_us = 0;  // 重置超时计时
    }
//    // 再处理res_bufB
    else if (res_bufB.ready) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			 sd_write_count++;
//      printf("SD写入操作次数: %lu\n", sd_write_count);
      WriteToSD(res_bufB.data, res_bufB.cnt);//31ms
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
      res_bufB.cnt = 0;
      res_bufB.ready = 0;
			res_bufB.start_us = 0;
    }
		 // 优先处理res_bufA
    if (res_bufC.ready) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			 sd_write_count++;
//      printf("SD写入操作次数: %lu\n", sd_write_count);
      WriteToSD(res_bufC.data, res_bufC.cnt);//31ms
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
      res_bufC.cnt = 0;        // 清空缓冲区
      res_bufC.ready = 0;      // 标记为未就绪
			res_bufC.start_us = 0;  // 重置超时计时
    }
//    // 再处理res_bufB
    else if (res_bufD.ready) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			 sd_write_count++;
//      printf("SD写入操作次数: %lu\n", sd_write_count);
      WriteToSD(res_bufD.data, res_bufD.cnt);//31ms
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
      res_bufD.cnt = 0;
      res_bufD.ready = 0;
			res_bufD.start_us = 0;
    }
		 // 优先处理res_bufA
    if (res_bufE.ready) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			 sd_write_count++;
//      printf("SD写入操作次数: %lu\n", sd_write_count);
      WriteToSD(res_bufE.data, res_bufE.cnt);//31ms
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
      res_bufE.cnt = 0;        // 清空缓冲区
      res_bufE.ready = 0;      // 标记为未就绪
			res_bufE.start_us = 0;  // 重置超时计时
    }
//    // 再处理res_bufB
    else if (res_bufF.ready) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			 sd_write_count++;
//      printf("SD写入操作次数: %lu\n", sd_write_count);
      WriteToSD(res_bufF.data, res_bufF.cnt);//31ms
			
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
      res_bufF.cnt = 0;
      res_bufF.ready = 0;
			res_bufF.start_us = 0;
    }
//		HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
 
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
