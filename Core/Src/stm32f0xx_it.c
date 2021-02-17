/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

extern int counter_top;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  counter_top++;
  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */
    counter_top++;

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void EXTI0_1_IRQHandler(void)
{
    /*
     * +1 implies forward state
     * -1 implies backward state
     *  0 implies transitional state
     */
    static int8_t states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    /*
     * Saves two last states
     */
    static uint8_t enc_trans = 0;
    counter_top++;
    /*
     * Stores sum of states giving direction
     */
    static int8_t enc_dir = 0;
    /*
     * To display current state
     */
    uint8_t enc_state = 0x00;

    enc_state = 0x0003 & LL_GPIO_ReadInputPort(GPIOA);
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    enc_dir += states[enc_trans];

    /*
     * If we got quite positive number it means there were more "+1"
     * hence it is forward direction
     */
    if (enc_dir == 4) {
        if (counter_top < 3000)
            counter_top += counter_top / 10;
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        enc_dir = 0;
    }

    /*
     * If we got quite negative number it means there were more "-1"
     * hence it is backward direction
     */
    if (enc_dir == -4) {
        if (counter_top > 10)
            counter_top -= counter_top / 10;
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
        enc_dir = 0;
    }

    /*
     * don't forget to reset flags
     */
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
