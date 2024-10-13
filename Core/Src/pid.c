                 // Device header

#include "pid.h"
#include "string.h"
#include "stdlib.h"
#include "main.h"
#include "stdio.h"

   CascadePID mypid;



void Pid_Init(PID* pid,float p,float i,float d,float maxI,float maxOut)
{
	pid->kp=p;
	pid->ki=i;
	pid->kd=d;
	
	pid->maxIntegral =maxI;
	pid->maxOutput =maxOut;
	
}
 
//进行一次pid计算
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//更新数据
    pid->lastError = pid->error; //将旧error存起来
    pid->error = reference - feedback; //计算新error
    //计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    float pout = pid->error * pid->kp;
	
    //计算积分
    pid->integral += pid->error * pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout+dout + pid->integral;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}
 


//串级PID的计算函数
//参数(PID结构体,外环目标值,外环反馈值,内环反馈值)
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid->outer, outerRef, outerFdb); //计算外环
    PID_Calc(&pid->inner, pid->outer.output, innerFdb); //计算内环
    pid->output = pid->inner.output; //内环输出就是串级PID的输出
}
 




//入参：字符串数组(存储ascii码数组)的地址 ; 字符串数组的长度
void get_PIDdata(uint8_t *data)
{
	int startIdx,endIdx;		//定义有效数据的起始索引和结束索引
	char valueStr[10] = {0}; 	//定义有效数据对应的字符串
	float PIDpara=0;		

	
	
		//找到 '=' 的索引
		for(int i=0;i<1000;i++)
		{
			if(data[i] == '=')
			{
				startIdx = i + 1;	//找到有效数据起始索引
				break;
			}
		}
		//找到 '!' 的索引
		for (int i = startIdx; i < 1000; i++)
        {
            if (data[i] == '!')
            {
                endIdx = i;		//找到有效数据结束索引
                break;
            }
        }
		//提取 '='与'!'之间的数值
		if (startIdx > 0 && endIdx > startIdx)
		{
			strncpy(valueStr, (char*)&data[startIdx], endIdx - startIdx);	//将有效数据长度的字符从data源字符串中拷贝到valueStr字符串中
			valueStr[endIdx - startIdx] = '\0';	//将valueStr字符串尾部补上'\0'，作为字符串结束标志
			PIDpara = atof(valueStr);		//将字符串转换为浮点数("2.32"-->2.32)
		}
		
	
		// 设置电机的PID参数
		if (data[0] == 'P' && data[1] == 'I')
		{
			mypid.inner.kp = PIDpara;
//		
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
			mypid.inner.maxIntegral = PIDpara;
			//PRINTF_PC("L_MaxIntegral = %.3f\n", speed_pid_L.maxIntegral);
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
			//PRINTF_PC("R_MaxIntegral = %.3f\n", speed_pid_R.maxIntegral);
		}
	}

      

