#include "string.h"
#include "stdlib.h"
#include "main.h"
#include "stdio.h"
#include "DR16.h"
#include "usart.h"

rc_info_t rc = {0};
uint8_t   dbus_buf[DBUS_BUFLEN];

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{//���ڽ������ݵĺ�����ʹ��DMA��ʽ������UART����
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	//����һ����ʱ����tmp1������UART�Ľ���״̬��ֵ����
	if (tmp1 == HAL_UART_STATE_READY)//�ж�UART�Ƿ��ھ���״̬
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//����DMA���գ�Դ��ַΪUART���ݼĴ�����Ŀ���ַΪ���ջ�����
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//ʹ��UART��DMA���չ���
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}


void dbus_uart_init(void)//DBUS���ڳ�ʼ��
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);//�������UART�Ŀ��б�־�����б�־ָʾUART�ڽ�������ʱ���ڿ���״̬��ͨ���ڽ�����ɺ�����
	//��������־��Ϊ��ȷ�������Ľ��ղ����ܹ���ȷ��⵽�µĿ���״̬
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//ʹ��UART�Ŀ����жϣ���UART���ڿ���״̬���ҽ��ջ�����û������ʱ���ᴥ������ж�
	//ʹ������жϺ󣬿������жϷ��������д������״̬
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);//����֮ǰ�ĺ�������DMA�����մ�������
}

void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{//���ڴ�����յ���ң������rc�����ݣ������յ����ֽڽ���
  rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;//��buff[0]��buff[1]��ֵ���Ϊch0ͨ����ֵ��������������11λ��ͨ����0x07FF��λ�룩
  rc->ch0 -= 1024;//����������364��1684�������������ݼ�ȥ1024��ʹ������ֵΪ0
  rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch3 -= 1024;
  rc->roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  //���Ͻǹ���
  rc->roll -= 1024;
 
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;//sw1��sw2��ֵ�ֱ�����Ӧ��λ����ó�
	
  if ((abs(rc->ch0) > 660)    || \
      (abs(rc->ch1) > 660)   || \
      (abs(rc->ch2) > 660)   || \
      (abs(rc->ch3) > 660))
	  
  {
    memset(rc, 0, sizeof(rc_info_t));//�����һͨ���ľ���ֵ����660����ʾ���յ��쳣���ݣ���rc�ṹ���������������
  }		
  
  
  if(rc->ch0 <= 5 && rc->ch0 >= -5)
    rc->ch0 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;

}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{//����DMAԤ����Ļ�����ʣ��ĳ��ȣ������˽⴫������л��ж���������δ����
  return ((uint16_t)(dma_stream->NDTR));
}
 
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//���UART�Ŀ��б�־���Ա���һ�ν���ʱ�ܹ���ȷ��⵽����״̬
	
	if (huart == &DBUS_HUART)//ȷ��ֻ����DBUS����
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//ʧ��DMA���գ���ֹ��һ�ν��յ���������һ�����ݵ�β����������ȫ�µ�����
 
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//���㵱ǰ���յ����ݳ��ȣ�������յ������ݳ��ȵ���18�ֽڣ�����ô������ݺ���
			rc_callback_handler(&rc, dbus_buf);	//������յ����ݲ�����
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//����DMA����Ԥ����Ļ������ĳ��ȣ��Ա�Ϊ��һ�ν�������׼��
		__HAL_DMA_ENABLE(huart->hdmarx);//��������DMA���գ��Ա������������
	}
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{//���ڼ��UART����״̬���ڽ��յ�����״̬ʱ������Ӧ�Ļص�����
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //���UART�Ƿ������˿��б�־����ʾUART������ɲ��������״̬
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//���UART�����ж��Ƿ�ʹ�ܣ�ֻ�����ж�ʹ�ܵ�����£��Żᴦ�����״̬
	{
		uart_rx_idle_callback(huart);//����֮ǰ����ĺ�����������յ�������
	}
}

//void USART2_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART3_IRQn 0 */
//	uart_receive_handler(&huart2);//����֮ǰ����ĺ���������DBUS���ڵĵ�ַ���Դ�������¼�
//  /* USER CODE END USART3_IRQn 0 */
//  HAL_UART_IRQHandler(&huart2);
//  /* USER CODE BEGIN USART3_IRQn 1 */
// 
//  /* USER CODE END USART3_IRQn 1 */
//}
