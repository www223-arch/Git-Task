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
uint8_t data[1000];//串口数据
CAN_TxHeaderTypeDef TxMessage;//发送报文结构体

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId dji_TaskHandle;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void dji_task(void const * argument);

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
  /* definition and creation of dji_Task */
  osThreadDef(dji_Task, dji_task, osPriorityNormal, 0, 128);
  dji_TaskHandle = osThreadCreate(osThread(dji_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_dji_task */
/**
  * @brief  Function implementing the dji_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_dji_task */
void dji_task(void const * argument)
{
  /* USER CODE BEGIN dji_task */
	
	can1_init();//can1初始化
	
  /* Infinite loop */
  for(;;)
  {
	  CAN1_Send_Test(TxMessage,1000);//发送控制电压
	  
     osDelay(1);
  }
  /* USER CODE END dji_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//can接收回调函数
 {
		uint8_t date_CAN1[8];//用于接收CAN1数据
	//	uint8_t date_CAN2[8];//用于接收CAN2数据
		
	
	if(hcan->Instance ==CAN1)
	{
		CAN_RxHeaderTypeDef RxHeader;  //创建接收报文结构体，只声明不配置
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, date_CAN1); //接收，CAN邮箱为0

		switch(RxHeader.StdId)
		{
	  case 0x205:
	{
    motor_yaw_info.rotor_angle    = ((date_CAN1[0] << 8) | date_CAN1[1]);
    motor_yaw_info.rotor_speed    = ((date_CAN1[2] << 8) | date_CAN1[3]);
    motor_yaw_info.torque_current = ((date_CAN1[4] << 8) | date_CAN1[5]);
    motor_yaw_info.temp           =   date_CAN1[6];
		break;	
	}
	
        }
	
    }
	
	else	if(hcan->Instance ==CAN2)
	{
		
	}
 
 
 }
 
 void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)//空闲中断回调函数
{    

		     

		
	HAL_UART_Transmit_DMA(&huart6,(uint8_t *)(data),Size);
		
	
   

	HAL_UARTEx_ReceiveToIdle_IT(&huart6,data,100); 
}

/* USER CODE END Application */
