#ifndef __MY_CAN_H
#define __MY_CAN_H



//#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/




 
typedef struct  //�����յ����Ϣ�Ľṹ��
{
    uint16_t can_id;//���ID
    int16_t  set_voltage;//�趨�ĵ�ѹֵ
    uint16_t rotor_angle;//��е�Ƕ�
    int16_t  rotor_speed;//ת��
    int16_t  torque_current;//Ť�ص���
    uint8_t  temp;//�¶�
}moto_info_t;
 

/* �������� */

void  can1_init(void);
void CAN1_Send_Test( CAN_TxHeaderTypeDef TxMessage,uint16_t data );
#endif
