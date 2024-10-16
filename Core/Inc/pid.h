#ifndef __PID_H
#define __PID_H

#include "cmsis_os.h"


typedef struct
{
   	float kp, ki, kd; //����ϵ��
    float error, lastError; //���ϴ����
    float integral, maxIntegral; //���֡������޷�
    float output, maxOutput; //���������޷�
	float key_num;
	float key;
}PID;


typedef struct
{
   	uint16_t encoder; //��ǰ��������ֵ
		
   uint16_t encoder_is_init;//�Ƿ�У׼
	uint16_t last_encoder;//�ϴα�������ֵ
	uint16_t encoder_offset;//������У׼���
	float angle_offset;//�Ƕ�У׼���
	float round_cnt;//Ȧ��total_encoder
	
	float angle;//ת���ĽǶ�
	float total_encoder;//ת���ı�����ֵ
}motor_angle;

typedef struct
{
    PID inner; //�ڻ�
    PID outer; //�⻷
    float output; //�������������inner.output
}CascadePID;
 


void get_PIDdata(uint8_t *data);
void Pid_Init(PID* pid,float p,float i,float d,float maxI,float maxOut,float num);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
void update_angle(motor_angle* _angle, uint16_t angle_fbk);




#endif
