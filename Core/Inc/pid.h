#ifndef __PID_H
#define __PID_H

#include "cmsis_os.h"


typedef struct
{
   	float kp, ki, kd; //����ϵ��
    float error, lastError; //���ϴ����
    float integral, maxIntegral; //���֡������޷�
    float output, maxOutput; //���������޷�
}PID;

typedef struct
{
    PID inner; //�ڻ�
    PID outer; //�⻷
    float output; //�������������inner.output
}CascadePID;
 


void get_PIDdata(uint8_t *data);
void Pid_Init(PID* pid,float p,float i,float d,float maxI,float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);





#endif
