#include "main.h"

#include "can.h"

#include "i2c.h"

#include "usart.h"
#include "gpio.h"
#include "MY_CAN.h"



moto_info_t  motor_yaw_info; 


void CAN1_Send_Test( CAN_TxHeaderTypeDef TxMessage,uint16_t data )
{
	
	uint32_t ID=0x1FF;  //控制报文ID
	uint32_t len=8;  //数据帧长度
	uint32_t CAN_TX_BOX=0;//发送邮箱
	uint8_t data_send[8]={0} ;
	
	 data_send[0] =data>>8;  //数据高八位
	 data_send[1] =data ;    //数据低八位
	

	//给can发送结构体赋值
	TxMessage.IDE= CAN_ID_STD; 
	TxMessage.StdId = ID;
	TxMessage.RTR =CAN_RTR_DATA;
	TxMessage.TransmitGlobalTime = DISABLE;
	TxMessage.DLC = len;
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage, data_send,&CAN_TX_BOX);
		
}

void CAN2_Send_Test( CAN_TxHeaderTypeDef TxMessage )
{
	
	uint32_t ID=0x2222;
	uint32_t len=4;
	uint32_t CAN_TX_BOX=0;
	uint8_t data[8] ={0x02,0x02,0x02,0x02};
	TxMessage.IDE= CAN_ID_STD;
	TxMessage.StdId = ID;
	TxMessage.RTR =CAN_RTR_DATA;
	TxMessage.TransmitGlobalTime = DISABLE;
	TxMessage.DLC = len;
	HAL_CAN_AddTxMessage(&hcan2,&TxMessage, data,&CAN_TX_BOX);
		
}


void  can1_init(void)
	
{

  MX_CAN1_Init();
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //三十二位
  sFilterConfig.FilterIdHigh = 0x0000;  //过滤器和掩码都给0，不过滤
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) ;

   
   
   	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) ;//使能中断

	
   
    HAL_CAN_Start(&hcan1);

	


}

void  can2_init(void)
	
{

	MX_CAN2_Init();



  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 14;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;//can2注意slbank比bank小


   HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) ;
   
   HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) ;
	
	
   HAL_CAN_Start(&hcan2);
	


}


