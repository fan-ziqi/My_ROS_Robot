#include "uart_db.h"
#include <stdio.h>

static uint8_t uart_db_flag_rx_ok = 0; //���ճɹ���־
static uint8_t uart_db_rx_con=0;       //���ռ�����
static uint8_t uart_db_rx_checksum;    //֡ͷ����У���
static uint8_t uart_db_rx_buf[40];     //���ջ��壬��������С�ڵ���32Byte
static uint8_t uart_db_tx_buf[40];     //���ͻ���

/**
  * @��  ��  UART DB���Դ��ڳ�ʼ��
  * @��  ��  baud�� ����������
  * @����ֵ	 ��
  */
void UART_DB_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//**���Դ���USART����******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //�򿪴���GPIO��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //�򿪴��������ʱ��
	
	//��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//����USART����
	USART_InitStructure.USART_BaudRate = baud; //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	//����USARTΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //�������ȼ�	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);//��ʼ������NVIC
	
	//ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	//ʹ�� USART�� �������
	USART_Cmd(USART1, ENABLE);
}

/**
  * @��  ��  DBUART �����жϷ�����
  * @��  ��  �� 
  * @����ֵ  ��
  */
void USART1_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Res =USART_ReceiveData(USART1);	
		
		if(uart_db_rx_con < 3)    //==����֡ͷ + ����
		{
			if(uart_db_rx_con == 0)  //����֡ͷ1 0xAA
			{
				if(Res == 0xAA)
				{
					uart_db_rx_buf[0] = Res;
					uart_db_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart_db_rx_con == 1) //����֡ͷ2 0x55
			{
				if(Res == 0x55)
				{
					uart_db_rx_buf[1] = Res;
					uart_db_rx_con = 2;
				}
				else
				{
					uart_db_rx_con = 0;						
				}				
			}
			else  //�������ݳ���
			{
				uart_db_rx_buf[2] = Res;
				uart_db_rx_con = 3;
				uart_db_rx_checksum = (0xAA+0x55) + Res;	//����У���
			}
		}
		else    //==��������
		{
			if(uart_db_rx_con < (uart_db_rx_buf[2]-1) )
			{
				uart_db_rx_buf[uart_db_rx_con] = Res;
				uart_db_rx_con++;
				uart_db_rx_checksum = uart_db_rx_checksum + Res;					
			}
			else    //�ж����1λ
			{
				//����У��
				if( Res == uart_db_rx_checksum )  //У����ȷ
				{	
					//�˴��������ݽ���
					uart_db_flag_rx_ok = 1;
					
					//������ɣ��ָ���ʼ״̬
					uart_db_rx_con = 0;					
				}
			}
		}
		
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	} 
}

/**
  * @��  ��  ��ȡ���յ�����
  * @��  ��  *pbuf����������ָ��,��1���ֽ�Ϊ֡���룬����Ϊ����
  * @����ֵ	 0-�����ݽ��գ�other-��Ҫ��ȡ�������ֽڸ���
  */
uint8_t UART_DB_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart_db_flag_rx_ok != 0)
	{
		cnt = uart_db_rx_buf[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = uart_db_rx_buf[3+i];
		}
		
		uart_db_flag_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

/**
  * @��  ��  UART �������ݣ�X-ProtocolЭ�飩
  * @��  ��  *pbuf����������ָ��
  *          len���������ݳ��ȸ�������27 (32-5)
  *          num��֡�ţ�֡����
  * @����ֵ	 ��
  */
void UART_DB_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
  uint8_t tx_checksum = 0;//����У���
	
	if(len <= 50) //32
	{
		/******��ȡ����******/
		uart_db_tx_buf[0] = 0xAA;    //֡ͷ
		uart_db_tx_buf[1] = 0x55;    //
		uart_db_tx_buf[2] = len+5;  //����������ȼ���֡����
		uart_db_tx_buf[3] = num;    //֡����
		
		for(i=0; i<len; i++)
		{
			uart_db_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******����У���******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart_db_tx_buf[i];
		}
		uart_db_tx_buf[i] = tx_checksum;
		
		
		/******��������******/	
		cnt = 5+len;
		
		//��ѯ���䷽ʽ
		for(i=0; i<cnt; i++)
		{
			USART_SendData(USART1, uart_db_tx_buf[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC) != SET);
		}
	}
}

/**************************���ڴ�ӡ��غ����ض���********************************/
/**
  * @��  ��  �ض���putc������USART1��	
  */
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}

	return ch;
}

/**
  * @��  ��  �ض���getc������USART1��	
  */
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
	{}

	return (int)USART_ReceiveData(USART1);
}

void usart1_send_char(u8 c)
{       
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
    USART_SendData(USART1,c);  
}
