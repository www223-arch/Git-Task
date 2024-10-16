                 // Device header

#include "pid.h"
#include "string.h"
#include "stdlib.h"
#include "main.h"
#include "stdio.h"

extern   CascadePID mypid;

#include <stdio.h>
#include <stdlib.h>
 
// abs(num));
 float num=0.3;
void Pid_Init(PID* pid,float p,float i,float d,float maxI,float maxOut,float num)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	pid->key_num=num;
	pid->maxIntegral =maxI;
	pid->maxOutput =maxOut;
	
}
 
//����һ��pid����
//����Ϊ(pid�ṹ��,Ŀ��ֵ,����ֵ)������������pid�ṹ���output��Ա��


void PID_Calc(PID *pid, float reference, float feedback)
{
	
	
 	//��������
    pid->lastError = pid->error; //����error������
	
	
    pid->error = reference - feedback; //������error
	
//	if( ((pid->error)<( pid->key_num))&&(-(pid->error)>(-( pid->key_num))))
//		
//	{
//     pid->key=1;
//	}		
//	
//	else {pid->key=0;}
	

    //����΢��
    float dout = (pid->error - pid->lastError) * pid->kd;
    //�������
    float pout = pid->error * pid->kp;
	
    //�������
    pid->integral += pid->error * pid->ki;
    //�����޷�
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
   // �������
    pid->output = pout+dout + pid->integral;
   // ����޷�
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
   else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}
 



//�ǶȻ��۸��º�����ע��λ��target�����ж�Ȧ��
void update_angle(motor_angle* _angle, uint16_t angle_fbk)
{
	_angle->encoder = angle_fbk;		
	if(_angle->encoder_is_init)
	{
		if(_angle->encoder - _angle->last_encoder > 4096)	//��ǰ��������Ƕ�-�ϴη����Ƕȳ�����Ȧ
		{																									//���ڽǶ�ֵΪ0-8292�����ýǶȵĻ��ۣ�Ҫ��Ȧ��������������ǰ����-�ϴη���ֵ�����ܴﵽĿ��
			_angle->round_cnt --;																	
		}
		if(_angle->encoder - _angle->last_encoder < -4096)	
		{
			_angle->round_cnt ++;														//++--���ò�ͬ����´���Ȧ
		}
	}
	else		//ִֻ��һ��
	{
		_angle->encoder_offset = _angle->encoder;		//��һ�εõ��ĽǶȷ�������encoder_offset����㣩
		_angle->encoder_is_init = 1;
	}
	_angle->angle_offset = (_angle->encoder_offset/8191.0f) * 360.0f;	//��е�Ƕ�ֵת��Ϊ�Ƕ�ֵ
	_angle->last_encoder = _angle->encoder;
	_angle->total_encoder = _angle->round_cnt*8191 + _angle->encoder - _angle->encoder_offset;	
	_angle->angle = _angle->total_encoder/8191.0f * 360.0f;				//����ת��
}

//����PID�ļ��㺯��
//����(PID�ṹ��,�⻷Ŀ��ֵ,�⻷����ֵ,�ڻ�����ֵ)
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid->outer, outerRef, outerFdb); //�����⻷
    PID_Calc(&pid->inner, pid->outer.output, innerFdb); //�����ڻ�
    pid->output = pid->inner.output; //�ڻ�������Ǵ���PID�����
}
 




//��Σ��ַ�������(�洢ascii������)�ĵ�ַ ; �ַ�������ĳ���
void get_PIDdata(uint8_t *data)
{
	int startIdx,endIdx;		//������Ч���ݵ���ʼ�����ͽ�������
	char valueStr[10] = {0}; 	//������Ч���ݶ�Ӧ���ַ���
	float PIDpara=0;		

	
	
		//�ҵ� '=' ������
		for(int i=0;i<1000;i++)
		{
			if(data[i] == '=')
			{
				startIdx = i + 1;	//�ҵ���Ч������ʼ����
				break;
			}
		}
		//�ҵ� '!' ������
		for (int i = startIdx; i < 1000; i++)
        {
            if (data[i] == '!')
            {
                endIdx = i;		//�ҵ���Ч���ݽ�������
                break;
            }
        }
		//��ȡ '='��'!'֮�����ֵ
		if (startIdx > 0 && endIdx > startIdx)
		{
			strncpy(valueStr, (char*)&data[startIdx], endIdx - startIdx);	//����Ч���ݳ��ȵ��ַ���dataԴ�ַ����п�����valueStr�ַ�����
			valueStr[endIdx - startIdx] = '\0';	//��valueStr�ַ���β������'\0'����Ϊ�ַ���������־
			PIDpara = atof(valueStr);		//���ַ���ת��Ϊ������("2.32"-->2.32)
		}
		
	
		// ���õ����PID����
		if (data[0] == 'P' && data[1] == 'I')
		{
			mypid.inner.kp = PIDpara;


			printf("PI=%f\n",PIDpara);
		}
		else if (data[0] == 'I' && data[1] == 'I')
		{
			mypid.inner.ki = PIDpara;
		
			printf("II=%f\r\n",PIDpara);
		}
		else if (data[0] == 'D' && data[1] == 'I')
		{
			mypid.inner.kd = PIDpara;
		
			printf("DI=%f\r\n",PIDpara);
		}
		else if (data[0] == 'M' && data[1] == 'I')
		{
			;
			mypid.inner.maxIntegral = PIDpara;
			printf("M+I=%f\r\n",PIDpara);
			
		}
		else if (data[0] == 'P' && data[1] == 'O')
		{
			mypid.outer.kp = PIDpara;
			
			printf("PO=%f\r\n",PIDpara);
		}
		else if (data[0] == 'I' && data[1] == 'O')
		{
			mypid.outer.ki = PIDpara;
			printf("IO=%f\r\n",PIDpara);
		}
		else if (data[0] == 'D' && data[1] == 'O')
		{
				mypid.outer.kd = PIDpara;
			printf("DO=%f\n",PIDpara);
		}
		else if (data[0] == 'M' && data[1] == 'O')
		{
		  mypid.outer.maxIntegral = PIDpara;
			
			
		}
	}

      

