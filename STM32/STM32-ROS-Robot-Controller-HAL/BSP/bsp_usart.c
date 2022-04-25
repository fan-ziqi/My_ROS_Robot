#include "main.h"
#include "bsp_usart.h"
#include "bsp_define.h"

#include <stdlib.h>

#define LOG_TAG    "USART"
#include "bsp_log.h"

// 消息头/消息尾定义
const unsigned char HEAD[2] = {0x55, 0xaa};
const unsigned char END[2] = {0x0d, 0x0a};

// 串口发送/接收联合体, data_transmit大小由data数据类型的字节数决定
union SerialMessageUnion
{
	short data;
	unsigned char data_transmit[2];
}SendUnion, ReceiveUnion;

extern UART_HandleTypeDef huart1;

static uint8_t uart_db_flag_rx_ok = 0; //接收成功标志
static uint8_t USART_Debug_RX_Count=0;       //接收计数器
static uint8_t USART_Debug_RX_Length=0;       //接收计数器
static uint8_t uart_db_rx_checksum;    //帧头部分校验和
static uint8_t USART_Debug_RX_Package[255];     //接收缓冲，数据内容小于等于32Byte
static uint8_t uart_db_tx_buf[40];     //发送缓冲

u8 USART_Debug_RX_Buffer[1];//HAL库使用的串口接收缓冲

int fputc(int c, FILE *stream)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);   
    return 1;
}

uint8_t ch_r;
int fgetc(FILE * F)    
{
    HAL_UART_Receive (&huart1,&ch_r,1,0xffff);//接收
    return ch_r;
}

void usart1_send_char(u8 c)
{       
	HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);   
}

extern char CMD_Buffer[];
extern int CMD_Buffer_Count;
extern int CMD_Flag;

// 消息格式: 
// 消息头1 消息头2 消息长度 消息码 [消息内容] 校验码 消息尾1 消息尾2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART1)
	{
		//换行符为0d0a
		if(USART_Debug_RX_Buffer[0] == 0x0d)
		{
			;
		}
		else if(USART_Debug_RX_Buffer[0] == 0x0a)
		{
			CMD_Flag = 1;
		}
		else
		{
			CMD_Buffer[CMD_Buffer_Count] = USART_Debug_RX_Buffer[0];		
			CMD_Buffer_Count++;
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // 重新开启接收中断
//	if (huart->Instance==USART1)
//	{
//		if(USART_Debug_RX_Count == 0) //==接收帧头 + 长度
//		{
//			
//		}
//		if(USART_Debug_RX_Count < 2) //==接收帧头 + 长度
//		{
//			if(USART_Debug_RX_Count == 0) //接收帧头1 0xAA
//			{
//				if(USART_Debug_RX_Buffer[0] == 0xAA)
//				{
//					USART_Debug_RX_Package[0] = USART_Debug_RX_Buffer[0];
//					USART_Debug_RX_Count = 1;
//				}
//				else
//				{
//					LOG_E("RECEIVE ERROR: HEAD1 is: 0x%02x, but it should be: 0x%02x\r\n", (int)USART_Debug_RX_Buffer[0], (int)HEAD[0]);
//				}
//			}
//			else if(USART_Debug_RX_Count == 1) //接收帧头2 0x55
//			{
//				if(USART_Debug_RX_Buffer[0] == 0x55)
//				{
//					USART_Debug_RX_Package[1] = USART_Debug_RX_Buffer[0];
//					USART_Debug_RX_Count = 2;
//				}
//				else
//				{
//					LOG_E("RECEIVE ERROR: HEAD2 is: 0x%02x, but it should be: 0x%02x\r\n", (int)USART_Debug_RX_Buffer[0], (int)HEAD[1]);
//					USART_Debug_RX_Count = 0;
//				}				
//			}
//		}
//		else if(USART_Debug_RX_Count == 2) //接收数据长度
//		{
//			USART_Debug_RX_Length = USART_Debug_RX_Buffer[0];
//			USART_Debug_RX_Package[2] = USART_Debug_RX_Length;
//			USART_Debug_RX_Count = 3;
//		}
//		else //接收数据
//		{//3 < length
//			if(USART_Debug_RX_Count < (USART_Debug_RX_Package[2]-1) )
//			{
//				USART_Debug_RX_Package[USART_Debug_RX_Count] = USART_Debug_RX_Buffer[0];
//				USART_Debug_RX_Count++;
//				uart_db_rx_checksum = uart_db_rx_checksum + USART_Debug_RX_Buffer[0];					
//			}
//			else    //判断最后1位
//			{
//				//数据校验
//				if( USART_Debug_RX_Buffer[0] == uart_db_rx_checksum )  //校验正确
//				{	
//					//此处进行数据解析
//					uart_db_flag_rx_ok = 1;
//					
//					//接收完成，恢复初始状态
//					USART_Debug_RX_Count = 0;					
//				}
//			}
//		}
//	} 
//  HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // 重新开启接收中断
}

/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：接收数据指针,第1个字节为帧编码，后面为数据
  * @返回值	 0-无数据接收，other-需要读取的数据字节个数
  */
uint8_t USART_DB_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart_db_flag_rx_ok != 0)
	{
		cnt = USART_Debug_RX_Package[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = USART_Debug_RX_Package[3+i];
		}
		
		uart_db_flag_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

// 串口发送函数
bool USART_Debug_Send(short * WriteData, unsigned char MessageLenth, unsigned int MessageCode)
{
	// 计算发送消息长度
  unsigned char SendLength = (unsigned int)(2 * MessageLenth) + 7;

	// 发送消息缓冲区(变长数组)
  unsigned char * SendBuf = (unsigned char *)malloc(SendLength * sizeof(unsigned char));

	// 消息头 SendBuf[0] SendBuf[1]
	for(int i = 0; i < 2; i++)
	{
			SendBuf[i] = HEAD[i];
	}

	// 消息长度 SendBuf[2]
	SendBuf[2] =  SendLength;

	// 消息码 SendBuf[3]
	SendBuf[3] = (unsigned char)MessageCode;
	
	// 消息内容 SendBuf[4] ~ SendBuf[4 + 2*MessageLenth - 1]
	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
	{
			SendUnion.data = WriteData[i]; //更新消息Union内容
			for(int j = 0; j < 2; j++)
			{
					//送入缓冲区
					SendBuf[i * 2 + 4 + j] = SendUnion.data_transmit[j];
			}
	}
	
	// 校验码计算 SendBuf[2*MessageLenth + 4]
	SendBuf[2*MessageLenth + 4] = CalCrc(SendBuf, 2*MessageLenth + 4); //校验MessageLenth消息+2消息头+1消息长度+1消息码=MessageLenth + 4

	// 消息尾 SendBuf[MessageLenth + 5] SendBuf[MessageLenth + 6]
	SendBuf[2*MessageLenth + 5] = END[0];
	SendBuf[2*MessageLenth + 6] = END[1];
	
	// DEBUG BEGIN
	LOG_D("Write:(DEC -> HEX) ");
	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
	{
			SendUnion.data = WriteData[i]; //更新消息Union内容
			LOG_D("%d -> 0x%02x 0x%02x",SendUnion.data, (unsigned int)SendUnion.data_transmit[0], (unsigned int)SendUnion.data_transmit[1]);
	}
	LOG_D("\r\nWrite:(ALL HEX) ");
	for(int i = 0; i < SendLength; i++)
	{
			LOG_D("0x%02x", (unsigned int)SendBuf[i]);
	}
	LOG_D("\r\nWrite: SendLength: %d", (unsigned int)SendLength);
	// DEBUG END

	// 串口发送消息
	HAL_UART_Transmit(&huart1, (uint8_t *)SendBuf, sizeof(SendBuf), 100);
	
	//消息Union清零
	SendUnion.data = 0; 

	return true;

}

// 8位循环冗余校验计算函数
unsigned char CalCrc(unsigned char * VectorData, unsigned short len)
{
    unsigned char crc;
    crc = 0;
    while(len--)
    {
        crc ^= (*VectorData++);
        for (int i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void USART_Init(void)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // 开启接收中断

	LOG_I("USART Init Success\r\n");
}
