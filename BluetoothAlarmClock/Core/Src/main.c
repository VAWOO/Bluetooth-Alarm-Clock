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
#include "adc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "lcm1602.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

RTC_TimeTypeDef aTime;
RTC_DateTypeDef aDate;

typedef enum
{
	NONE,
	UP,
	DOWN,
	RIGHT,
	LEFT
}_Direction;

typedef enum
{
	AMPM=0,
	HOUR_T,
	HOUR_O,
	MINUTE_T,
	MINUTE_O,
	SECOND_T,
	SECOND_O,
}_SetMode;

typedef enum
{
	BUTTERFLY=0,
	LITTLESTAR,
	JINGLEBELLS
}_Belltype;

typedef enum
{
	SETTING,
	NORMAL,
	ALARM,
	BELL
}_MODE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RTC_FLASH_ADDRESS 0x08060000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
_MODE mode=SETTING;

// AM/PM
_SetMode setmode = AMPM;
char tmpTime[100]= {0,};
char ampm[2][3] = {"AM", "PM"};

// Bell
_Belltype belltype = BUTTERFLY;
char tmp_bell_name[20]={0,};
char bell_name[3][20]={ {">> Butterfly    "}, {">> Little Star  "}, {">> Jingle Bells "}, };

enum notes {C = 956, D = 852, E = 758, F = 716, G = 638, A = 568, B = 506};
uint16_t butterfly[] = {G, E, E, F, D, D, C, D, E, F, G, G, G, G, E, E, E, F, D, D, C, E, G, G, E, E, E, D, D, D, D, D, E, F, E, E, E, E, E, F, G, G, E, E, E, F, D, D, C, E, G, G, E, E, E};
uint16_t butterfly_interval[] = {10, 10, 400, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 800};
uint8_t butterfly_length = sizeof(butterfly)/sizeof(uint16_t);

uint16_t littlestar[] = {C, C, G, G, A, A, G, F, F, E, E, D, D, C, G, G, F, F, E, E, D, G, G, F, F, E, E, D, C, C, G, G, A, A, G, F, F, E, E, D, D, C};
uint16_t littlestar_interval[] = {10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 400, 10, 10, 10, 10, 10, 10, 800};
uint8_t littlestar_length = sizeof(littlestar)/sizeof(uint16_t);

uint16_t jinglebells[] = {E, E, E, E, E, E, E, G, C, D, E, F, F, F, F, F, E, E, E, E, D, D, E, D, G, E, E, E, E, E, E, E, G, C, D, E, F, F, F, F, F, E, E, E, G, G, F, D, C};
uint16_t jinglebells_interval[] = {10, 10, 400, 10, 10, 400, 10, 10, 10, 10, 800, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 400, 600, 10, 10, 400, 10, 10, 400, 10, 10, 10, 10, 800, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 800};
uint8_t jinglebells_length = sizeof(jinglebells) / sizeof(uint16_t);

// Button
_Direction direction;
_Direction button;
_Direction Bl_button;

uint8_t user_pressed_flag=0;
uint8_t user_pulled_flag=0;

uint32_t old_tick=0;
uint32_t current_tick=0;

uint8_t double_click = 0;
uint32_t interval = 0;
uint32_t interval_chk[2] = {0, };
int pulled_chk = -1;

// Alarm
uint8_t alarm_on=0;

// Joystick
uint32_t XY[2];

// Bluetooth
uint8_t rx2_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
_Direction getButton();

void move_cur_time(RTC_TimeTypeDef *time, _Direction direction);
void move_cur_bell(_Direction direction);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  LCM1602_init();

  HAL_UART_Receive_IT(&huart2, &rx2_data, sizeof(rx2_data));

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  HAL_ADC_Start_DMA(&hadc1, XY, 2);

  uint8_t toggle=0;

  char customChar[] = {0x01, 0x03, 0x05, 0x09, 0x09, 0x0B, 0x1B, 0x18};
  LCD_SendCommand(LCD_ADDR, 0x40);
  for(int i=0; i<8; i++) LCD_SendData(customChar[i]);

  uint32_t readRtcData[3];
  FLASH_Read(RTC_FLASH_ADDRESS, readRtcData, sizeof(readRtcData) / sizeof(readRtcData[0]));
  sTime.Hours = readRtcData[0];
  sTime.Minutes = readRtcData[1];
  sTime.Seconds = readRtcData[2];
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // setting mode
	  if(mode==SETTING)
	  {
		  toggle^=1;

		  lcd_put_cur(0,0);
		  LCD_SendString("Setting Mode    ");

		  // Read Button
		  button = getButton();
		  move_cur_time(&sTime, button);

		  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

		  // the part where it's big and blinking
		  if(toggle)
		  {
			  sprintf(tmpTime,"%s %02d:%02d:%02d ", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes, sTime.Seconds);
		  }
		  else
		  {
			  if(setmode==AMPM)
			  {
				  sprintf(tmpTime,"   %02d:%02d:%02d ", sTime.Hours, sTime.Minutes, sTime.Seconds);
			  }
			  else if(setmode==HOUR_T)
			  {
				  sprintf(tmpTime,"%s  %d:%02d:%02d ", ampm[sTime.TimeFormat], sTime.Hours%10, sTime.Minutes, sTime.Seconds);
			  }
			  else if(setmode==HOUR_O)
			  {
				  sprintf(tmpTime,"%s %d :%02d:%02d ", ampm[sTime.TimeFormat], sTime.Hours/10, sTime.Minutes, sTime.Seconds);
			  }
			  else if(setmode==MINUTE_T)
			  {
				  sprintf(tmpTime,"%s %02d: %d:%02d ", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes%10, sTime.Seconds);
			  }
			  else if(setmode==MINUTE_O)
			  {
				  sprintf(tmpTime,"%s %02d:%d :%02d ", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes/10, sTime.Seconds);
			  }
			  else if(setmode==SECOND_T)
			  {
				  sprintf(tmpTime,"%s %02d:%02d: %d ", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes, sTime.Seconds%10);
			  }
			  else if(setmode==SECOND_O)
			  {
				  sprintf(tmpTime,"%s %02d:%02d:%d ", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes, sTime.Seconds/10);
			  }
		  }

		  lcd_put_cur(1,0);
		  LCD_SendString(tmpTime);

		  if(user_pressed_flag==1)
		  {
			  current_tick=HAL_GetTick();

			  if(current_tick-old_tick > 1)
			  {
				  old_tick=current_tick;
				  user_pressed_flag=0;
				  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				  mode=NORMAL;

				  uint32_t rtcData[3] = {sTime.Hours, sTime.Minutes, sTime.Seconds};
				  FLASH_Write(RTC_FLASH_ADDRESS, rtcData, sizeof(rtcData) / sizeof(rtcData[0]));
			  }
		  }
	  }
	  // normal mode
	  else if(mode==NORMAL)
	  {
		  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		  sprintf(tmpTime,"%s %02d:%02d:%02d     ", ampm[sTime.TimeFormat], sTime.Hours, sTime.Minutes, sTime.Seconds);
		  lcd_put_cur(0,0);
		  LCD_SendString("Choi Jin Woo    ");
		  lcd_put_cur(1,0);
		  LCD_SendString(tmpTime);


		  if(sTime.TimeFormat==aTime.TimeFormat && sTime.Hours==aTime.Hours && sTime.Minutes==aTime.Minutes && sTime.Seconds==aTime.Seconds)
		  {
			  alarm_on=1;
		  }

		  if(alarm_on==1)
		  {
			  if (belltype == BUTTERFLY)
			  {
				  toggle^=1;

				  if(toggle==1)
				  {
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				  }
				  else
				  {
					  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

					  for(int i=0; i < butterfly_length; i++)
					  {
					  		  TIM2->ARR = butterfly[i];
					  		  TIM2->CCR1 = TIM2->ARR / 2;
					  		  HAL_Delay(500);
					  		  TIM2->CCR1 = 0;
					  		  HAL_Delay(butterfly_interval[i]);
					  		  TIM2->CCR1 = TIM2->ARR / 2;
					  }

					  alarm_on=0;
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				  }
			  }
			  else if (belltype == LITTLESTAR)
			  {
				  toggle^=1;

				  if(toggle==1)
				  {
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				  }
				  else
				  {
					  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

					  for(int i=0; i < littlestar_length; i++)
					  {
					  		  TIM2->ARR = littlestar[i];
					  		  TIM2->CCR1 = TIM2->ARR / 2;
					  		  HAL_Delay(500);
					  		  TIM2->CCR1 = 0;
					  		  HAL_Delay(littlestar_interval[i]);
					  		  TIM2->CCR1 = TIM2->ARR / 2;
					  }

					  alarm_on=0;
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				  }
			  }
			  else if (belltype == JINGLEBELLS)
			  {
				  toggle^=1;

				  if(toggle==1)
				  {
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				  }
				  else
				  {
					  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

					  for(int i=0; i < jinglebells_length; i++)
					  {
					  		  TIM2->ARR = jinglebells[i];
					  		  TIM2->CCR1 = TIM2->ARR / 2;
					  		  HAL_Delay(500);
					  		  TIM2->CCR1 = 0;
					  		  HAL_Delay(jinglebells_interval[i]);
					  		  TIM2->CCR1 = TIM2->ARR / 2;
					  }

					  alarm_on=0;
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				  }
			  }
		  }

		  if(user_pressed_flag==1)
		  {
			  current_tick=HAL_GetTick();

			  if(current_tick-old_tick > 2000)
			  {
				  old_tick=current_tick;
				  user_pressed_flag=0;
				  mode=ALARM;
				  HAL_RTC_GetTime(&hrtc, &aTime, RTC_FORMAT_BIN);
				  HAL_RTC_GetDate(&hrtc, &aDate, RTC_FORMAT_BIN);
			  }
		  }

		  if(double_click==1)
		  {
			  uint32_t rtcData[3] = {sTime.Hours, sTime.Minutes, sTime.Seconds};
			  FLASH_Write(RTC_FLASH_ADDRESS, rtcData, sizeof(rtcData) / sizeof(rtcData[0]));

			  mode=BELL;
			  double_click=0;
		  }
	  }
	  // alarm mode
	  else if(mode==ALARM)
	  {
		  toggle^=1;

		  lcd_put_cur(0,0);
		  LCD_SendString("Alarm Mode      ");

		  button = getButton();
		  move_cur_time(&aTime, button);

		  if(toggle)
		  {
			  sprintf(tmpTime,"%s %02d:%02d:%02d   ", ampm[aTime.TimeFormat], aTime.Hours, aTime.Minutes, aTime.Seconds);
		  }
		  else
		  {
			  if(setmode==AMPM)
			  {
				  sprintf(tmpTime,"   %02d:%02d:%02d ", aTime.Hours, aTime.Minutes, aTime.Seconds);
			  }
			  else if(setmode==HOUR_T)
			  {
				  sprintf(tmpTime,"%s  %d:%02d:%02d ", ampm[aTime.TimeFormat], aTime.Hours%10, aTime.Minutes, aTime.Seconds);
			  }
			  else if(setmode==HOUR_O)
			  {
				  sprintf(tmpTime,"%s %d :%02d:%02d ", ampm[aTime.TimeFormat], aTime.Hours/10, aTime.Minutes, aTime.Seconds);
			  }
			  else if(setmode==MINUTE_T)
			  {
				  sprintf(tmpTime,"%s %02d: %d:%02d ", ampm[aTime.TimeFormat], aTime.Hours, aTime.Minutes%10, aTime.Seconds);
			  }
			  else if(setmode==MINUTE_O)
			  {
				  sprintf(tmpTime,"%s %02d:%d :%02d ", ampm[aTime.TimeFormat], aTime.Hours, aTime.Minutes/10, aTime.Seconds);
			  }
			  else if(setmode==SECOND_T)
			  {
				  sprintf(tmpTime,"%s %02d:%02d: %d ", ampm[aTime.TimeFormat], aTime.Hours, aTime.Minutes, aTime.Seconds%10);
			  }
			  else if(setmode==SECOND_O)
			  {
				  sprintf(tmpTime,"%s %02d:%02d:%d ", ampm[aTime.TimeFormat], aTime.Hours, aTime.Minutes, aTime.Seconds/10);
			  }
		  }

		  lcd_put_cur(1,0);
		  LCD_SendString(tmpTime);
		  lcd_put_cur(1, 14);
		  LCD_SendData(0);

		  if(user_pressed_flag==1)
		  {
			  current_tick=HAL_GetTick();

			  if(current_tick-old_tick > 1)
			  {
				  old_tick=current_tick;
				  user_pressed_flag=0;
				  pulled_chk = -1;
				  mode=NORMAL;
			  }
		  }
	  }
	  else if (mode == BELL)
	  {
		  lcd_put_cur(0, 0);
		  LCD_SendString("Select Bell     ");

		  button = getButton();
		  move_cur_bell(button);

		  sprintf(tmp_bell_name, "%s", bell_name[belltype]);

		  lcd_put_cur(1, 0);
		  LCD_SendString(tmp_bell_name);

		  if (user_pressed_flag == 1)
		  {
			  current_tick = HAL_GetTick();
			  if (current_tick - old_tick > 1)
			  {
				  old_tick = current_tick;
				  user_pressed_flag = 0;
				  interval = 0;
				  pulled_chk = -1;
				  mode = NORMAL;
			  }
		  }
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
_Direction getButton()
{
    if (Bl_button != NONE)
    {
        _Direction temp = Bl_button;
        Bl_button = NONE;
        return temp;
    }

	if(XY[0] < 300)
		return UP;
	else if(XY[0] > 3800)
		return DOWN;
	else if(XY[1] > 3800)
		return LEFT;
	else if(XY[1] < 300)
		return RIGHT;
	else
		return NONE;
}

void move_cur_time(RTC_TimeTypeDef *Time, _Direction direction)
{
	switch(direction)
	{
	case RIGHT:
		setmode++;
		if(setmode > SECOND_O) setmode = SECOND_O;
		break;
	case LEFT:
		if(setmode > AMPM) setmode--;
		break;
	case UP:
		if(setmode==AMPM)
		{
			Time->TimeFormat ^= 1;
		}
		else if(setmode==HOUR_T)
		{
			Time->Hours+=10;
			if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 1;
		}
		else if(setmode==HOUR_O)
		{
			Time->Hours++;
			if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 1;
		}
		else if(setmode==MINUTE_T)
		{
			Time->Minutes+=10;
			if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 0;
		}
		else if(setmode==MINUTE_O)
		{
			Time->Minutes++;
			if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 0;
		}
		else if(setmode==SECOND_T)
		{
			Time->Seconds+=10;
			if(!IS_RTC_SECONDS(Time->Seconds)) Time->Seconds = 0;
		}
		else if(setmode==SECOND_O)
		{
			Time->Seconds++;
			if(!IS_RTC_SECONDS(Time->Seconds)) Time->Seconds = 0;
		}
		break;
	case DOWN:
		if(setmode==AMPM)
		{
			Time->TimeFormat ^= 1;
		}
		else if(setmode==HOUR_T)
		{
			Time->Hours-=10;
			if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 12;
		}
		else if(setmode==HOUR_O)
		{
			Time->Hours--;
			if(!IS_RTC_HOUR12(Time->Hours)) Time->Hours = 12;
		}
		else if(setmode==MINUTE_T)
		{
			Time->Minutes-=10;
			if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 59;
		}
		else if(setmode==MINUTE_O)
		{
			Time->Minutes--;
			if(!IS_RTC_MINUTES(Time->Minutes)) Time->Minutes = 59;
		}
		else if(setmode==SECOND_T)
		{
			Time->Seconds-=10;
			if(!IS_RTC_SECONDS(Time->Seconds)) Time->Seconds = 59;
		}
		else if(setmode==SECOND_O)
		{
			Time->Seconds--;
			if(!IS_RTC_SECONDS(Time->Seconds)) Time->Seconds = 59;
		}
		break;
	case NONE:
		break;
	}
}

void move_cur_bell(_Direction direction)
{
	switch(direction)
	{
	case RIGHT:
		break;
	case LEFT:
		break;
	case NONE:
		break;
	case UP:
		if(belltype < 2) belltype++;
		else belltype=0;
		break;
	case DOWN:
		if(belltype > 0) belltype--;
		else belltype=2;
		break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
		{
			user_pulled_flag = 0;
			user_pressed_flag = 1;
			old_tick=HAL_GetTick();
			current_tick=HAL_GetTick();

			if (mode == NORMAL)
			{
				interval_chk[0] = HAL_GetTick();

				interval = interval_chk[0] - interval_chk[1];
			}
		}
		else
		{
			user_pulled_flag = 1;
			user_pressed_flag = 0;

			if (mode == NORMAL)
			{
				interval_chk[1] = HAL_GetTick();

				pulled_chk++;

				if(interval > 0 && interval < 300 && pulled_chk > 1)
				{
					double_click = 1;
				}
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART2)
	{
		HAL_UART_Transmit(&huart3, &rx2_data, sizeof(rx2_data), 10);
		HAL_UART_Receive_IT(&huart2, &rx2_data, sizeof(rx2_data));

		switch(rx2_data)
		{
		case 'U':
			Bl_button = UP;
			break;
		case 'D':
			Bl_button = DOWN;
			break;
		case 'L':
			Bl_button = LEFT;
			break;
		case 'R':
			Bl_button = RIGHT;
			break;
		case 'E':
			if (mode != NORMAL)
				user_pressed_flag=1;
			break;
		case 'B':
			if (mode == NORMAL)
				double_click=1;
			break;
		case 'A':
			if (mode == NORMAL)
				user_pressed_flag=1;
			break;
		default:
			break;
		}
	}
}
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
