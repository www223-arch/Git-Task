#ifndef __MY_CAN_H
#define __MY_CAN_H



//#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/




 
typedef struct  //定义收电机信息的结构体
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}moto_info_t;
 

/* 函数声明 */

void  can1_init(void);
void CAN1_Send_Test( CAN_TxHeaderTypeDef TxMessage,uint16_t data );
#endif
