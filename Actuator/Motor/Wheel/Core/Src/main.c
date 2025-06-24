/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t ADCdata[3];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define WS_H           136   // 1 码相对计数值
#define WS_L           59   // 0 码相对计数值
#define WS_REST        40   // 复位信号脉冲数量
#define LED_NUM         4  // WS2812灯个数
#define DATA_LEN       24   // WS2812数据长度，单个需要24个字节
#define WS2812_RST_NUM 50   // 官方复位时间为50us（40个周期），保险起见使用50个周期

//显存数组，长度为 灯的数量*24+复位周期
uint16_t WS2812_RGB_Buff[LED_NUM*DATA_LEN+WS2812_RST_NUM] = {0}; 
 
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    //指针偏移:需要跳过复位信号的N个0
    uint16_t* p = (WS2812_RGB_Buff + WS2812_RST_NUM) + (num * LED_NUM);
    
    for (uint16_t i = 0;i < 8;i++)
    {
        //填充数组
        p[i]      = (G << i) & (0x80)?WS_H : WS_L;
        p[i + 8]  = (R << i) & (0x80)?WS_H : WS_L;
        p[i + 16] = (B << i) & (0x80)?WS_H : WS_L;
    }
}

// HSV到RGB转换函数
void HSVtoRGB(int h, int s, int v, uint8_t *r, uint8_t *g, uint8_t *b) {
    int h_i = (h%360)/60;
    int f = h - 60*h_i;
    int p = (v*(255-s))>>8;
    int q = (v*(255-((s*f)>>8)))>>8;
    int t = (v*(255-((s*(255-f))>>8)))>>8;
    
    switch(h_i) {
        case 0: *r=v; *g=t; *b=p; break;
        case 1: *r=q; *g=v; *b=p; break;
        case 2: *r=p; *g=v; *b=t; break;
        case 3: *r=p; *g=q; *b=v; break;
        case 4: *r=t; *g=p; *b=v; break;
        case 5: *r=v; *g=p; *b=q; break;
    }
}
//WS2812初始化函数
void WS2812_Init()
{
	//设置关闭所有灯
  // for(int i=0;i<LED_NUM;i++)
  // {
	//   WS2812_Set(i,0,20,0);
  // }
  //作用：调用DMA将显存中的内容实时搬运至定时器的比较寄存器
  HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_1,(uint32_t *)WS2812_RGB_Buff,sizeof(WS2812_RGB_Buff)/sizeof(uint16_t)); 
}

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[64];
uint8_t RxData[64];

void FDCAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len) {
  TxHeader.Identifier = id;              // CAN ID
  TxHeader.IdType = FDCAN_STANDARD_ID;   // 标准ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 数据长度编码
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;     // 关闭速率切换（经典模式）
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;      // 经典CAN帧
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
			// printf("ID: %X Data: %X  %X \r\n", RxHeader.Identifier, RxData[0], RxData[7]);
    }	
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {  // 
   loop();
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    torqueLoop();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

  if(hadc==&hadc1)
  {
	  // printf("%f %f %d\r\n",((float)data[0]/4096)*3.0*16,temp_trans(data[1]),data[2]);//temp_trans(data[1])
    //   HAL_UART_Transmit(&huart2, (uint8_t*)message,sizeof(message),HAL_MAX_DELAY);
    //  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)data,3);

  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);//校准
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADCdata,3);
 
  
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  WS2812_Init();
  // 启用RX FIFO0中断
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  // 启动FDCAN
  HAL_FDCAN_Start(&hfdcan1);
  uint8_t r, g, b, v = 0, toggle = 0;
  setup();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    logLoop();
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADCdata,3);
    HSVtoRGB(120, 255, v, &r, &g, &b);
    ws2812_set_RGB(r/2, g/2, b/2, 0);
    if (toggle == 0)
    {
     v++;
     if (v == 250)
     {
      toggle = 1;
     }
     
    }
    else
    {
      v--;
     if (v == 5)
     {
      toggle = 0;
     }
    }

    // uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    // FDCAN_SendMessage(0x123, data, 8);
    // HAL_Delay(100);
    // printf("%f %f %d\r\n",((float)ADCdata[0]/4096)*3.0*16,temp_trans(ADCdata[1]),ADCdata[2]);
    HAL_Delay(10);
    // printf("test\r\n");
    //HAL_UART_Transmit(&huart1, data, 3, 100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
