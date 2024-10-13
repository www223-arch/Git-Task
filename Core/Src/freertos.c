/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_CAN.h"

#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern  moto_info_t  motor_yaw_info; //收反馈报文结构体
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId pwm_TaskHandle;
osThreadId DJI_send_taskHandle;
osThreadId serial_send_tasHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void pwm_task(void const * argument);
void DJI_send(void const * argument);
void serial_send(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of pwm_Task */
  osThreadDef(pwm_Task, pwm_task, osPriorityNormal, 0, 128);
  pwm_TaskHandle = osThreadCreate(osThread(pwm_Task), NULL);

  /* definition and creation of DJI_send_task */
  osThreadDef(DJI_send_task, DJI_send, osPriorityIdle, 0, 128);
  DJI_send_taskHandle = osThreadCreate(osThread(DJI_send_task), NULL);

  /* definition and creation of serial_send_tas */
  osThreadDef(serial_send_tas, serial_send, osPriorityIdle, 0, 128);
  serial_send_tasHandle = osThreadCreate(osThread(serial_send_tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_pwm_task */
/**
  * @brief  Function implementing the pwm_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pwm_task */
void pwm_task(void const * argument)
{
  /* USER CODE BEGIN pwm_task */
	
	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
		
	 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  /* Infinite loop */
  for(;;)
  {
	  
	   __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 150); 
	   __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 150);
	  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 150); 
	   __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 150);
	  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 150); 
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 150); 
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 150); 
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 150); 	  
	  
	  
    osDelay(1);
  }
  /* USER CODE END pwm_task */
}

/* USER CODE BEGIN Header_DJI_send */
/**
* @brief Function implementing the DJI_send_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJI_send */
void DJI_send(void const * argument)
{
  /* USER CODE BEGIN DJI_send */
	CAN_TxHeaderTypeDef TxMessage;
  /* Infinite loop */
  for(;;)
  {
	CAN1_Send_Test(TxMessage,10000);
    osDelay(1);
  }
  /* USER CODE END DJI_send */
}

/* USER CODE BEGIN Header_serial_send */
/**
* @brief Function implementing the serial_send_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serial_send */
void serial_send(void const * argument)
{
  /* USER CODE BEGIN serial_send */
  /* Infinite loop */
  for(;;)
  {
	  printf("%s\n","wl");
	  
    osDelay(1);
  }
  /* USER CODE END serial_send */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



/* USER CODE END Application */
