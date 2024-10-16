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
#include "DR16.h"
#include "gpio.h"
#include "MPU6050.h"
#include "stdio.h"
#include "MY_CAN.h"
#include "PID.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern  moto_info_t  motor_yaw_info; //收反馈报文结构体
	
CAN_TxHeaderTypeDef TxMessage;
CascadePID mypid = {0}; //创建串级PID结构体变量

//uint8_t data[1000];//串口变量

float angleTarget,Feedback;//角度目标和编码器反馈

//float  num1,num2,num3,num4;

float outerTarget;//外环目标
float outerFeedback;//外环反馈
float innerFeedback;//内环反馈

motor_angle Angle={0};//角度更新变量

float pitch, roll, yaw;

extern rc_info_t rc ;
extern uint8_t   dbus_buf[DBUS_BUFLEN];

float outerTarget;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId serial_TaskHandle;
osThreadId DJI_pid_taskHandle;
osThreadId mpu_TaskHandle;
osThreadId DR16_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void seria_task(void const * argument);
void DJI_pid(void const * argument);
void Mpu_tTask(void const * argument);
void Start_DR16_Task(void const * argument);

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
  /* definition and creation of serial_Task */
  osThreadDef(serial_Task, seria_task, osPriorityNormal, 0, 384);
  serial_TaskHandle = osThreadCreate(osThread(serial_Task), NULL);

  /* definition and creation of DJI_pid_task */
  osThreadDef(DJI_pid_task, DJI_pid, osPriorityIdle, 0, 384);
  DJI_pid_taskHandle = osThreadCreate(osThread(DJI_pid_task), NULL);

  /* definition and creation of mpu_Task */
  osThreadDef(mpu_Task, Mpu_tTask, osPriorityIdle, 0, 384);
  mpu_TaskHandle = osThreadCreate(osThread(mpu_Task), NULL);

  /* definition and creation of DR16_Task */
  osThreadDef(DR16_Task, Start_DR16_Task, osPriorityIdle, 0, 384);
  DR16_TaskHandle = osThreadCreate(osThread(DR16_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_seria_task */
/**
  * @brief  Function implementing the serial_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_seria_task */
void seria_task(void const * argument)
{
  /* USER CODE BEGIN seria_task */
  /* Infinite loop */
  for(;;)
  {
	  
	  	//HAL_UARTEx_ReceiveToIdle_IT(&huart6,data,100); //开启串口中断，调参
	
    osDelay(1);
  }
  /* USER CODE END seria_task */
}

/* USER CODE BEGIN Header_DJI_pid */
/**
* @brief Function implementing the DJI_pid_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJI_pid */
void DJI_pid(void const * argument)
{
  /* USER CODE BEGIN DJI_pid */
	
	
	can1_init();
	
   Pid_Init(&mypid.inner, 39, 15, 0, 25000, 25000,50); //初始化内环参数
  // Pid_Init(&mypid.outer,30, 0,0, 8000, 8191,30); //初始化外环参数

  /* Infinite loop */
  for(;;)
  {
	
		 taskENTER_CRITICAL() ;
	  

			innerFeedback = motor_yaw_info.rotor_speed ;////获取内环反馈值
		//	outerFeedback = motor_yaw_info.rotor_angle ; //获取外环反馈值	  	  	  
			//rc.ch3*1.0	  
	  
	 
	  
		//	update_angle(&Angle,outerFeedback);//角度更新

			//PID_Calc(&(mypid.outer),angleTarget,Angle.angle);	  //外环计算

			PID_Calc(&(mypid.inner),rc.ch3*1.0/660*330	,innerFeedback);//内环计算
			  
			CAN1_Send_Test(TxMessage,mypid.inner.output);//发送串级计算值
		    
          taskEXIT_CRITICAL() ;
		  
		 // printf("%f,%f\n",outerTarget,Angle.angle );//搭配vofa绘制曲线
		  printf("%f,%f\n",rc.ch3*1.0/660*330,innerFeedback );//搭配vofa绘制曲线
		  
    osDelay(1);
  }
  /* USER CODE END DJI_pid */
}

/* USER CODE BEGIN Header_Mpu_tTask */
/**
* @brief Function implementing the mpu_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mpu_tTask */
void Mpu_tTask(void const * argument)
{
  /* USER CODE BEGIN Mpu_tTask */
// taskENTER_CRITICAL() ;
//int ret;
//do
//{
//ret = MPU6050_DMP_init();
//}

//while(ret);
// taskEXIT_CRITICAL() ;
  /* Infinite loop */
  for(;;)
  {
//	 taskENTER_CRITICAL() ;
//	  
//	     MPU6050_DMP_Get_Date(&pitch, &roll, &yaw);//获取角度
//	  
//	  taskEXIT_CRITICAL() ;
//		 printf("%f\n pitch",pitch );
//		 printf("%f\n roll",roll );
//		 printf("%f\n yaw",yaw );//打印
	  
       osDelay(1);
  }
  /* USER CODE END Mpu_tTask */
}

/* USER CODE BEGIN Header_Start_DR16_Task */
/**
* @brief Function implementing the DR16_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DR16_Task */
void Start_DR16_Task(void const * argument)
{
  /* USER CODE BEGIN Start_DR16_Task */
	
  dbus_uart_init();
	
//  HAL_UARTEx_ReceiveToIdle_IT(&huart6,dbus_buf,100);
  /* Infinite loop */
  for(;;)
  {
//		printf("%f\n ch0",rc.ch0*1.0 );
//		printf("%d\n ch1",rc.ch1 );
//		printf("%d\n ch2",rc.ch2 );//打印
//		printf("%f\n ch3",rc.ch3*1.0  );
//		printf("%d\n roll",rc.roll );
//		printf("%d\n sw1",rc.sw1 );
//		printf("%d\n sw2",rc.sw2 );
	//HAL_UARTEx_ReceiveToIdle_IT(&huart2,data,100); 
	  
    osDelay(1);
  }
  /* USER CODE END Start_DR16_Task */
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

/* USER CODE END Application */
