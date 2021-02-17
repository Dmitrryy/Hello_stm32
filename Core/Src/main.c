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

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"

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

int counter_top = 7777;

#define A_INDICATOR_ID_PORT 11
#define B_INDICATOR_ID_PORT 12
#define C_INDICATOR_ID_PORT 13
#define D_INDICATOR_ID_PORT 14
#define E_INDICATOR_ID_PORT 15
#define F_INDICATOR_ID_PORT 16
#define G_INDICATOR_ID_PORT 17
#define DP_INDICATOR_ID_PORT 18

#define ANODE1_INDICATOR_ID_PORT 19
#define ANODE2_INDICATOR_ID_PORT 20
#define ANODE3_INDICATOR_ID_PORT 21
#define ANODE4_INDICATOR_ID_PORT 22



#define LED0_PORT GPIOA, LL_GPIO_PIN_1
#define LED1_PORT GPIOA, LL_GPIO_PIN_2
#define LED2_PORT GPIOA, LL_GPIO_PIN_3
#define LED3_PORT GPIOF, LL_GPIO_PIN_4
#define LED4_PORT GPIOF, LL_GPIO_PIN_5
#define LED5_PORT GPIOA, LL_GPIO_PIN_4
#define LED6_PORT GPIOA, LL_GPIO_PIN_5
#define LED7_PORT GPIOA, LL_GPIO_PIN_6
#define LED8_PORT GPIOA, LL_GPIO_PIN_7
#define LED9_PORT GPIOC, LL_GPIO_PIN_4


#define LED0_ON() LL_GPIO_SetOutputPin(LED0_PORT)
#define LED0_OFF() LL_GPIO_ResetOutputPin(LED0_PORT)
#define LED1_ON() LL_GPIO_SetOutputPin(LED1_PORT)
#define LED1_OFF() LL_GPIO_ResetOutputPin(LED1_PORT)
#define LED2_ON() LL_GPIO_SetOutputPin(LED2_PORT)
#define LED2_OFF() LL_GPIO_ResetOutputPin(LED2_PORT)
#define LED3_ON() LL_GPIO_SetOutputPin(LED3_PORT)
#define LED3_OFF() LL_GPIO_ResetOutputPin(LED3_PORT)
#define LED4_ON() LL_GPIO_SetOutputPin(LED4_PORT)
#define LED4_OFF() LL_GPIO_ResetOutputPin(LED4_PORT)
#define LED5_ON() LL_GPIO_SetOutputPin(LED5_PORT)
#define LED5_OFF() LL_GPIO_ResetOutputPin(LED5_PORT)
#define LED6_ON() LL_GPIO_SetOutputPin(LED6_PORT)
#define LED6_OFF() LL_GPIO_ResetOutputPin(LED6_PORT)
#define LED7_ON() LL_GPIO_SetOutputPin(LED7_PORT)
#define LED7_OFF() LL_GPIO_ResetOutputPin(LED7_PORT)
#define LED8_ON() LL_GPIO_SetOutputPin(LED8_PORT)
#define LED8_OFF() LL_GPIO_ResetOutputPin(LED8_PORT)
#define LED9_ON() LL_GPIO_SetOutputPin(LED9_PORT)
#define LED9_OFF() LL_GPIO_ResetOutputPin(LED9_PORT)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void switchOutPortMood(uint32_t id, int mood);
void indicatorDisplay(uint16_t num);

void EXTI0_1_IRQHandler(void);

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

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    indicatorDisplay(counter_top);


    /*  {
          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);
          LED3_OFF();
          LED4_ON();
          LL_mDelay(100);
          LED4_OFF();
          LED5_ON();
          LL_mDelay(100);
          LED5_OFF();
          LED6_ON();
          LL_mDelay(100);
          LED6_OFF();
          LED7_ON();
          LL_mDelay(100);
          LED7_OFF();
          LED8_ON();
          LL_mDelay(100);
          LED8_OFF();
          LED9_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);
          LED3_OFF();
          LED4_ON();
          LL_mDelay(100);
          LED4_OFF();
          LED5_ON();
          LL_mDelay(100);
          LED5_OFF();
          LED6_ON();
          LL_mDelay(100);
          LED6_OFF();
          LED7_ON();
          LL_mDelay(100);
          LED7_OFF();
          LED8_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);
          LED3_OFF();
          LED4_ON();
          LL_mDelay(100);
          LED4_OFF();
          LED5_ON();
          LL_mDelay(100);
          LED5_OFF();
          LED6_ON();
          LL_mDelay(100);
          LED6_OFF();
          LED7_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);
          LED3_OFF();
          LED4_ON();
          LL_mDelay(100);
          LED4_OFF();
          LED5_ON();
          LL_mDelay(100);
          LED5_OFF();
          LED6_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);
          LED3_OFF();
          LED4_ON();
          LL_mDelay(100);
          LED4_OFF();
          LED5_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);
          LED3_OFF();
          LED4_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);
          LED2_OFF();
          LED3_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);
          LED1_OFF();
          LED2_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);
          LED0_OFF();
          LED1_ON();
          LL_mDelay(100);

          LED0_ON();
          LL_mDelay(100);

          LED0_OFF();
          LED1_OFF();
          LED2_OFF();
          LED3_OFF();
          LED4_OFF();
          LED5_OFF();
          LED6_OFF();
          LED7_OFF();
          LED8_OFF();
          LED9_OFF();
          LL_mDelay(100);
      }*/


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(48000000);
  LL_SetSystemCoreClock(48000000);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);

  /**/
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7);

  /**/
  LL_GPIO_ResetOutputPin(LD4_GPIO_Port, LD4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void indicatorDisplay(uint16_t num)
{
    /*
 * 026
       4-разрядный 7-сегментный индикатор
                1  A  F 2  3  B
 ______________|__|__|__|__|__|__________
|    A    |    A    |    A    |    A    |
| F     B | F     B | F     B | F     B |
|    G    |    G    |    G    |    G    |
| E     C | E     C | E     C | E     C |
|    D  dp|    D  dp|    D  dp|    D  dp|
|_________|_________|_________|_________|
                |  |  |  |  |  |
                E  D dp  C  G  4

Соответствие отображаемого знака данным порта
общий анод
__________________________________________________
     |   двоичный вид по сегментам   |            |
Цифра|dp | G | F | E | D | C | B | A | Десятичный |
-----|---|---|---|---|---|---|---|---|------------|
  0  | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |    192     |
  1  | 1 | 1 | 1 | 1 | 1 | 0 | 0 | 1 |    249     |
  2  | 1 | 0 | 1 | 0 | 0 | 1 | 0 | 0 |    164     |
  3  | 1 | 0 | 1 | 1 | 0 | 0 | 0 | 0 |    176     |
  4  | 1 | 0 | 0 | 1 | 1 | 0 | 0 | 1 |    153     |
  5  | 1 | 0 | 0 | 1 | 0 | 0 | 1 | 0 |    146     |
  6  | 1 | 0 | 0 | 0 | 0 | 0 | 1 | 0 |    130     |
  7  | 1 | 1 | 1 | 1 | 1 | 0 | 0 | 0 |    248     |
  8  | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |    128     |
  9  | 1 | 0 | 0 | 1 | 0 | 0 | 0 | 0 |    144     |
 dp  | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |    127     |
-----*---*---*---*---*---*---*---*---*------------*

 */

    uint8_t digitMatrix[10][7] = {
           // A, B, C, D, E, F, G
            { 1, 1, 1, 1, 1, 1, 0 }, //0
            { 0, 1, 1, 0, 0, 0, 0 }, //1
            { 1, 1, 0, 1, 1, 0, 1 }, //2
            { 1, 1, 1, 1, 0, 0, 1 }, //3
            { 0, 1, 1, 0, 0, 1, 1 }, //4
            { 1, 0, 1, 1, 0, 1, 1 }, //5
            { 1, 0, 1, 1, 1, 1, 1 }, //6
            { 1, 1, 1, 0, 0, 0, 0 }, //7
            { 1, 1, 1, 1, 1, 1, 1 }, //8
            { 1, 1, 1, 0, 0, 1, 1 }, //9
    };

    const uint16_t timeLight = 3;

    const uint16_t n1 = (uint16_t)(num / 1000) % 10;
    const uint16_t n2 = (uint16_t)(num / 100) % 10;
    const uint16_t n3 = (uint16_t)(num / 10) % 10;
    const uint16_t n4 = num % 10;

    switchOutPortMood(ANODE1_INDICATOR_ID_PORT, 0);
    switchOutPortMood(ANODE2_INDICATOR_ID_PORT, 1);
    switchOutPortMood(ANODE3_INDICATOR_ID_PORT, 1);
    switchOutPortMood(ANODE4_INDICATOR_ID_PORT, 1);

    switchOutPortMood(A_INDICATOR_ID_PORT, digitMatrix[n1][0]);
    switchOutPortMood(B_INDICATOR_ID_PORT, digitMatrix[n1][1]);
    switchOutPortMood(C_INDICATOR_ID_PORT, digitMatrix[n1][2]);
    switchOutPortMood(D_INDICATOR_ID_PORT, digitMatrix[n1][3]);
    switchOutPortMood(E_INDICATOR_ID_PORT, digitMatrix[n1][4]);
    switchOutPortMood(F_INDICATOR_ID_PORT, digitMatrix[n1][5]);
    switchOutPortMood(G_INDICATOR_ID_PORT, digitMatrix[n1][6]);
    LL_mDelay(timeLight);


    switchOutPortMood(ANODE1_INDICATOR_ID_PORT, 1);
    switchOutPortMood(ANODE2_INDICATOR_ID_PORT, 0);

    switchOutPortMood(A_INDICATOR_ID_PORT, digitMatrix[n2][0]);
    switchOutPortMood(B_INDICATOR_ID_PORT, digitMatrix[n2][1]);
    switchOutPortMood(C_INDICATOR_ID_PORT, digitMatrix[n2][2]);
    switchOutPortMood(D_INDICATOR_ID_PORT, digitMatrix[n2][3]);
    switchOutPortMood(E_INDICATOR_ID_PORT, digitMatrix[n2][4]);
    switchOutPortMood(F_INDICATOR_ID_PORT, digitMatrix[n2][5]);
    switchOutPortMood(G_INDICATOR_ID_PORT, digitMatrix[n2][6]);
    LL_mDelay(timeLight);


    switchOutPortMood(ANODE2_INDICATOR_ID_PORT, 1);
    switchOutPortMood(ANODE3_INDICATOR_ID_PORT, 0);

    switchOutPortMood(A_INDICATOR_ID_PORT, digitMatrix[n3][0]);
    switchOutPortMood(B_INDICATOR_ID_PORT, digitMatrix[n3][1]);
    switchOutPortMood(C_INDICATOR_ID_PORT, digitMatrix[n3][2]);
    switchOutPortMood(D_INDICATOR_ID_PORT, digitMatrix[n3][3]);
    switchOutPortMood(E_INDICATOR_ID_PORT, digitMatrix[n3][4]);
    switchOutPortMood(F_INDICATOR_ID_PORT, digitMatrix[n3][5]);
    switchOutPortMood(G_INDICATOR_ID_PORT, digitMatrix[n3][6]);
    LL_mDelay(timeLight);


    switchOutPortMood(ANODE3_INDICATOR_ID_PORT, 1);
    switchOutPortMood(ANODE4_INDICATOR_ID_PORT, 0);

    switchOutPortMood(A_INDICATOR_ID_PORT, digitMatrix[n4][0]);
    switchOutPortMood(B_INDICATOR_ID_PORT, digitMatrix[n4][1]);
    switchOutPortMood(C_INDICATOR_ID_PORT, digitMatrix[n4][2]);
    switchOutPortMood(D_INDICATOR_ID_PORT, digitMatrix[n4][3]);
    switchOutPortMood(E_INDICATOR_ID_PORT, digitMatrix[n4][4]);
    switchOutPortMood(F_INDICATOR_ID_PORT, digitMatrix[n4][5]);
    switchOutPortMood(G_INDICATOR_ID_PORT, digitMatrix[n4][6]);
    LL_mDelay(timeLight);

}



void switchOutPortMood(uint32_t id, int mood)
{
    void* typePort = 0;
    unsigned int port = 0;

    switch(id)
    {
        case A_INDICATOR_ID_PORT:
            typePort = GPIOB;
            port = LL_GPIO_PIN_15;
            break;

        case B_INDICATOR_ID_PORT:
            typePort = GPIOC;
            port = LL_GPIO_PIN_7;
            break;

        case C_INDICATOR_ID_PORT:
            typePort = GPIOA;
            port = LL_GPIO_PIN_11;
            break;

        case D_INDICATOR_ID_PORT:
            typePort = GPIOA;
            port = LL_GPIO_PIN_9;
            break;

        case E_INDICATOR_ID_PORT:
            typePort = GPIOA;
            port = LL_GPIO_PIN_8;
            break;

        case F_INDICATOR_ID_PORT:
            typePort = GPIOC;
            port = LL_GPIO_PIN_6;
            break;

        case G_INDICATOR_ID_PORT:
            typePort = GPIOA;
            port = LL_GPIO_PIN_12;
            break;

        case DP_INDICATOR_ID_PORT:
            typePort = GPIOA;
            port = LL_GPIO_PIN_10;
            break;

        case ANODE1_INDICATOR_ID_PORT:
            typePort = GPIOC;
            port = LL_GPIO_PIN_5;
            break;

        case ANODE2_INDICATOR_ID_PORT:
            typePort = GPIOB;
            port = LL_GPIO_PIN_0;
            break;

        case ANODE3_INDICATOR_ID_PORT:
            typePort = GPIOB;
            port = LL_GPIO_PIN_1;
            break;

        case ANODE4_INDICATOR_ID_PORT:
            typePort = GPIOB;
            port = LL_GPIO_PIN_2;
            break;
    }

    if (mood == 0) {
        LL_GPIO_ResetOutputPin(typePort, port);
    }
    else if (mood == 1) {
        LL_GPIO_SetOutputPin(typePort, port);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
