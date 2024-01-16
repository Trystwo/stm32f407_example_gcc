/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "led.h"
#include "usart.h"
#include "sram.h"
#include "lcd.h"
#include "delay.h"
#include "key.h"
#include "malloc.h"
#include "lvgl_demo.h"
#include "touch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
//void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

//uint8_t str[250000] __attribute__((section(".sram_data" )));
uint8_t str[16];
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
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
  delay_init(168);                    /* 初始化延时函数 */
  led_init();
  usart_init(115200);
  lcd_init();
  key_init();
  sram_init();
  tp_dev.init();
  my_mem_init(SRAMIN);                /* 初始化内部内存池 */
  my_mem_init(SRAMEX);                /* 初始化外部内存池 */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  //lvgl_demo();                        /* 运行FreeRTOS例程 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_OscInitTypeDef rcc_osc_init = {0};
  RCC_ClkInitTypeDef rcc_clk_init = {0};

  __HAL_RCC_PWR_CLK_ENABLE();                                         /* 使能PWR时钟 */

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);      /* 设置调压器输出电压级别，以便在器件未以最大频率工作 */

  /* 使能HSE，并选择HSE作为PLL时钟源，配置PLL1，开启USB时钟 */
  rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;        /* 时钟源为HSE */
  rcc_osc_init.HSEState = RCC_HSE_ON;                          /* 打开HSE */
  rcc_osc_init.PLL.PLLState = RCC_PLL_ON;                      /* 打开PLL */
  rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;              /* PLL时钟源选择HSE */
  rcc_osc_init.PLL.PLLN = plln;
  rcc_osc_init.PLL.PLLM = pllm;
  rcc_osc_init.PLL.PLLP = pllp;
  rcc_osc_init.PLL.PLLQ = pllq;
  ret = HAL_RCC_OscConfig(&rcc_osc_init);                      /* 初始化RCC */
  if(ret != HAL_OK)
  {
    return 1;                                                /* 时钟初始化失败，可以在这里加入自己的处理 */
  }

  /* 选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2 */
  rcc_clk_init.ClockType = ( RCC_CLOCKTYPE_SYSCLK \
                                | RCC_CLOCKTYPE_HCLK \
                                | RCC_CLOCKTYPE_PCLK1 \
                                | RCC_CLOCKTYPE_PCLK2);

  rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;         /* 设置系统时钟时钟源为PLL */
  rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;                /* AHB分频系数为1 */
  rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV4;                 /* APB1分频系数为4 */
  rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV2;                 /* APB2分频系数为2 */
  ret = HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_5);   /* 同时设置FLASH延时周期为5WS，也就是6个CPU周期 */
  if(ret != HAL_OK)
  {
    return 1;                                                /* 时钟初始化失败 */
  }

  /* STM32F405x/407x/415x/417x Z版本的器件支持预取功能 */
  if (HAL_GetREVID() == 0x1001)
  {
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();                    /* 使能flash预取 */
  }
  return 0;
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
