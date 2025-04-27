/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "config.h"
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

/* External variables ov7670--------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dcmi;
extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;


/* External variables keypad--------------------------------------------------------*/
extern int8_t current_col;
extern GPIO_TypeDef* rowPorts[];
extern uint16_t rowPins[];

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	if (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_TC)) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}




/**
  * @brief This function handles EXTI line 4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  // Your code here
	char message[100];
	
	if (__HAL_GPIO_EXTI_GET_FLAG(COL0_Pin)) {
		/*
		if (HAL_GPIO_ReadPin(rowPorts[0], rowPins[0]) == GPIO_PIN_SET){
			sprintf(message, "3");
			print_msg(message);
		}
		else if(HAL_GPIO_ReadPin(rowPorts[1], rowPins[1]) == GPIO_PIN_SET){
			sprintf(message, "7");
			print_msg(message);
		}
		*/
		if(HAL_GPIO_ReadPin(rowPorts[2], rowPins[2]) == GPIO_PIN_SET){
			sprintf(message, "B");
			sobel_flag = 1;
			print_msg(message);
		}
		else if(HAL_GPIO_ReadPin(rowPorts[3], rowPins[3]) == GPIO_PIN_SET){
			sprintf(message, "F");
			print_msg(message);
		}
  }

  HAL_GPIO_EXTI_IRQHandler(COL0_Pin);
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  // Your code here
	char message[100];
	
	if (__HAL_GPIO_EXTI_GET_FLAG(COL1_Pin)) {
			/*
			if(HAL_GPIO_ReadPin(rowPorts[0], rowPins[0]) == GPIO_PIN_SET){
				sprintf(message, "2");
				print_msg(message);
			}
			else if(HAL_GPIO_ReadPin(rowPorts[1], rowPins[1]) == GPIO_PIN_SET){
				sprintf(message, "6");
				print_msg(message);
			}
			*/
			if(HAL_GPIO_ReadPin(rowPorts[2], rowPins[2]) == GPIO_PIN_SET){
				sprintf(message, "A");
				dma_flag = 1;
				print_msg(message);
			}
			else if(HAL_GPIO_ReadPin(rowPorts[3], rowPins[3]) == GPIO_PIN_SET){
				sprintf(message, "E");
				print_msg(message);
			}
		}
	
	else if (__HAL_GPIO_EXTI_GET_FLAG(COL2_Pin)) {
			/*
			if(HAL_GPIO_ReadPin(rowPorts[0], rowPins[0]) == GPIO_PIN_SET){
				sprintf(message, "1");
				print_msg(message);
			}
			else if(HAL_GPIO_ReadPin(rowPorts[1], rowPins[1]) == GPIO_PIN_SET){
				sprintf(message, "5");
				print_msg(message);
			}
		*/
			if(HAL_GPIO_ReadPin(rowPorts[2], rowPins[2]) == GPIO_PIN_SET){
				sprintf(message, "9");
				print_msg(message);
			}
			else if(HAL_GPIO_ReadPin(rowPorts[3], rowPins[3]) == GPIO_PIN_SET){
				sprintf(message, "D");
				print_msg(message);
			}
		}
	else if (__HAL_GPIO_EXTI_GET_FLAG(COL3_Pin)) {
		/*
			if(HAL_GPIO_ReadPin(rowPorts[0], rowPins[0]) == GPIO_PIN_SET){
				sprintf(message, "0");
				print_msg(message);
			}
			else if(HAL_GPIO_ReadPin(rowPorts[1], rowPins[1]) == GPIO_PIN_SET){
				sprintf(message, "4");
				print_msg(message);
			}
		*/
			if(HAL_GPIO_ReadPin(rowPorts[2], rowPins[2]) == GPIO_PIN_SET){
				sprintf(message, "8");
				print_msg(message);
			}
			else if(HAL_GPIO_ReadPin(rowPorts[3], rowPins[3]) == GPIO_PIN_SET){
				sprintf(message, "C");
				print_msg(message);
			}
		}

  HAL_GPIO_EXTI_IRQHandler(COL1_Pin);
  HAL_GPIO_EXTI_IRQHandler(COL2_Pin);
  HAL_GPIO_EXTI_IRQHandler(COL3_Pin);
}





/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    if (__HAL_GPIO_EXTI_GET_FLAG(USER_Btn_Pin)) 
    {
        // Clear the interrupt flag
        __HAL_GPIO_EXTI_CLEAR_IT(USER_Btn_Pin);

        // Set the flag to indicate a button press
        dma_flag = 1;

        // Optional: Toggle an LED for debugging
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        // Optional: Print a debug message
        print_msg("BUTTON PRESSED\n");
    }
    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(USER_Btn_Pin);
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
	
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dcmi);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */
	
  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DCMI global interrupt.
  */
void DCMI_IRQHandler(void)
{
  /* USER CODE BEGIN DCMI_IRQn 0 */
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dcmi);
  /* USER CODE END DCMI_IRQn 0 */
  HAL_DCMI_IRQHandler(&hdcmi);
  /* USER CODE BEGIN DCMI_IRQn 1 */

  /* USER CODE END DCMI_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
