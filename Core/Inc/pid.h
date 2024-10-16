#ifndef __PID_H
#define __PID_H

#include "cmsis_os.h"


typedef struct
{
   	float kp, ki, kd; //三个系数
    float error, lastError; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
	float key_num;
	float key;
}PID;


typedef struct
{
   	uint16_t encoder; //当前编码器的值
		
   uint16_t encoder_is_init;//是否校准
	uint16_t last_encoder;//上次编码器的值
	uint16_t encoder_offset;//编码器校准零点
	float angle_offset;//角度校准零点
	float round_cnt;//圈数total_encoder
	
	float angle;//转过的角度
	float total_encoder;//转过的编码器值
}motor_angle;

typedef struct
{
    PID inner; //内环
    PID outer; //外环
    float output; //串级输出，等于inner.output
}CascadePID;
 


void get_PIDdata(uint8_t *data);
void Pid_Init(PID* pid,float p,float i,float d,float maxI,float maxOut,float num);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
void update_angle(motor_angle* _angle, uint16_t angle_fbk);




#endif
