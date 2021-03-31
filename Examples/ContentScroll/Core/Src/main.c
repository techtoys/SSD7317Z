/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SSD7317.h"
#include "rpc.h"
#include "tone.h"
#ifdef USE_FULL_ASSERT
#include <stdio.h>
#endif
#include "ArialBlack_36h.h"
#include "android.h"
#include "facebook.h"
#include "instagram.h"
#include "linkedin.h"
#include "youtube.h"
#include "battery-status-full.h"
#include "Tahoma_12h.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void app_touch_task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief
 * \b Description:<br>
 * 	This function redirect printf() to UART2 port, i.e. VCP port.
 * \b Pre-requisite:<br>
 *  1. Define a new symbol #USE_FULL_ASSERT under
 *  	Project Properties>C/C++ General>Paths and Symbols>#Symbols>GNU C<br>
 *  2. Need two files: syscalls.c and sysmem.c to work. They are automatically
 *  	generated by STM32CubeIDE if you choose Nucleo Standard EVK as the hw platform
 */
#ifdef USE_FULL_ASSERT
/* @note Override __io_putchar() in syscalls.c */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
#endif

#define ICON_MAX 5
const tIcon icons[ICON_MAX] = {
		{0, "android", &android},
		{1, "facebook", & facebook},
		{2, "instagram", & instagram},
		{3, "linkedin", &linkedin},
		{4, "youtube", &youtube},
};

#define LABEL_X 0
#define LABEL_Y 10
#define ICON_X	16
#define ICON_Y	64
#define BATTERY_X	72
#define BATTERY_Y	0

void app_touch_task(void)
{
	static bool sleep = false;
	static bool por = true;
	static bool cons_scroll = false;
	static uint8_t icon_index = 0;
	static char str[3];
	static uint16_t label_w, label_h;

	if(por){
		snprintf(str, 3, "%d", icon_index);
		por = false;
		ssd7317_put_image(BATTERY_X,BATTERY_Y,&batterystatusfull,0);
		ssd7317_put_string(LABEL_X,LABEL_Y,&ArialBlack_36h,str,0);
		ssd7317_get_stringsize(&Tahoma_12h, icons[icon_index].name, &label_w, &label_h);
		ssd7317_put_string(OLED_HOR_RES-label_w, ICON_Y-label_h-6, &Tahoma_12h, icons[icon_index].name, 0);
		ssd7317_put_image(ICON_X,ICON_Y, icons[icon_index].image, 0);
	}
	finger_t finger = ssd7317_get_gesture();
	switch(finger.act){
	case LONG_TAP_ANYKEY:
			ssd7317_enter_lpm();
			tone_pwm_set(500); tone_pwm_on();
			HAL_Delay(700); tone_pwm_off();
			sleep = true;
			break;
	case DOUBLE_TAP_ANYKEY:
		if(sleep){
			ssd7317_display_on();
			sleep=false;
		}
		break;
	case SINGLE_TAP_ANYKEY:
		if(cons_scroll){
			ssd7317_cons_scroll_brake();
			cons_scroll=false;
			tone_pwm_off();
		}else{
			tone_pwm_set(1000);
			tone_pwm_on();
			ssd7317_put_image(ICON_X,ICON_Y, icons[icon_index].image, 1);
			tone_pwm_set(500);
			HAL_Delay(50);
			ssd7317_put_image(ICON_X,ICON_Y, icons[icon_index].image, 0);
			tone_pwm_off();
		}
		break;
	case SWIPE:
		if(!sleep){
			if(finger.tap_down_key!=finger.tap_up_key){
				tone_pwm_set(500);
				//tone_pwm_on();

				//clear the icon's name background first
				rect_t string_bg = {OLED_HOR_RES-label_w, ICON_Y-label_h-6, OLED_HOR_RES-1, ICON_Y-7};
				ssd7317_fill_color(string_bg, BLACK);

				//increment / decrement on the icon index
				if(finger.detail==SWIPE_DOWN){
					icon_index++;
					icon_index%=ICON_MAX;
				}else{
					icon_index--;
					if(icon_index>ICON_MAX-1) icon_index=ICON_MAX-1;
				}

				//update the icon's name
				ssd7317_get_stringsize(&Tahoma_12h, icons[icon_index].name, &label_w, &label_h);
				ssd7317_put_string(OLED_HOR_RES-label_w, ICON_Y-label_h-6, &Tahoma_12h, icons[icon_index].name, 0);

				/**
				 * Scrolling icon by graphic command: ssd7317_cntnt_scroll_image() -or-
				 * scroll by frame buffer linearly: ssd7317_linear_scroll_image() -or-
				 * scroll with animation like a spring: ssd7317_spring_scroll_image()
				 */
				//ssd7317_cntnt_scroll_image(ICON_X,ICON_Y,icons[icon_index].image,finger);
				//ssd7317_linear_scroll_image(ICON_X,ICON_Y,2,icons[icon_index].image,finger);
				ssd7317_spring_scroll_image(ICON_X,ICON_Y,2,icons[icon_index].image,finger);

				//update the icon index
				snprintf(str, 3, "%d", icon_index);
				ssd7317_put_string(LABEL_X,LABEL_Y,&ArialBlack_36h,str,0);

				//tone_pwm_off();
			}else{
				rect_t full_page={0,0,OLED_HOR_RES-1,OLED_VER_RES-1};
				ssd7317_cons_scroll_page(full_page,7,5,finger);
				cons_scroll = true;
				tone_pwm_set(200);
				tone_pwm_on();
			}
		}
		break;
	case LARGE_OBJ:
	case ACT_ERROR:
		/* Your action to handle large object or act error*/
		break;
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

  /* USER CODE BEGIN 2 */
  tone_pwm_init();
  ssd7317_init();
  rpc_uart_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  app_touch_task();
	  rpc_main_task();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
