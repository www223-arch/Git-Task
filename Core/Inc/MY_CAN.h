#ifndef __MY_CAN_H
#define __MY_CAN_H



//#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/


/* º¯ÊýÉùÃ÷ */
void  can2_init(void);
void  can1_init(void);
void CAN1_Send_Test( CAN_TxHeaderTypeDef TxMessage );
void CAN2_Send_Test( CAN_TxHeaderTypeDef TxMessage );


#endif
