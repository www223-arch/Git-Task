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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_CAN.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId can_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Can_Task(void const * argument);

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
  /* definition and creation of can_Task */
  osThreadDef(can_Task, Can_Task, osPriorityNormal, 0, 128);
  can_TaskHandle = osThreadCreate(osThread(can_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Can_Task */
/**
  * @brief  Function implementing the can_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Can_Task */
void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
	  HAL_Init();

	can1_init();
    can2_init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C1_Init();
    MX_USART6_UART_Init();


CAN_TxHeaderTypeDef TxMessage;
  /* Infinite loop */
  for(;;)
  {

		CAN2_Send_Test( TxMessage);
		osDelay(1000);
		CAN1_Send_Test( TxMessage);
		osDelay(1000);


  }
  /* USER CODE END Can_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint8_t date_CAN1[8];//设为全局变量，用于接收CAN1数据
 uint8_t date_CAN2[8];//设为全局变量，用于接收CAN2数据
 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance ==CAN1)
	{
	  CAN_RxHeaderTypeDef RxHeader;  //创建接收报文结构体，只声明不配置
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, date_CAN1); //接收，CAN邮箱为0
	  HAL_UART_Transmit_DMA(&huart6,(uint8_t *)(date_CAN1),4);
		
	 
	}
	 if(hcan->Instance == CAN2)
	{
	  CAN_RxHeaderTypeDef RxHeader;  //创建接收报文结构体，只声明不配置
	  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, date_CAN2); //接收，CAN邮箱为0
		  HAL_UART_Transmit_DMA(&huart6,(uint8_t *)(date_CAN2),4);
		
	}
}


/* USER CODE END Application */
