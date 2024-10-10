#include "main.h"

#include "can.h"

#include "i2c.h"

#include "usart.h"
#include "gpio.h"
#include "MY_CAN.h"

//CAN_HandleTypeDef   g_can1_handler;     /* CAN控制句柄 */
//CAN_TxHeaderTypeDef g_can1_txheader;    /* CAN发送结构体 */
//CAN_RxHeaderTypeDef g_can1_rxheader;    /* CAN接收结构体 */

//static CAN_TxHeaderTypeDef TxMessage;
//static CAN_RxHeaderTypeDef RxMessage;

void CAN1_Send_Test( CAN_TxHeaderTypeDef TxMessage )
{
	
	uint32_t ID=0x1222;
	uint32_t len=4;
	uint32_t CAN_TX_BOX=0;
	uint8_t data[8] ={0x01,0x01,0x01,0x01};
	TxMessage.IDE= CAN_ID_STD;
	TxMessage.StdId = ID;
	TxMessage.RTR =CAN_RTR_DATA;
	TxMessage.TransmitGlobalTime = DISABLE;
	TxMessage.DLC = len;
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage, data,&CAN_TX_BOX);
		
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
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) ;

   
   
   	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) ;

	
   
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

//接受
/**
 * @brief  CAN FIFO0的中断回调函数，在里面完成数据的接收
 * @param  hcan     CAN的句柄
 */
