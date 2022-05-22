/*
	Copyright 2022 Fan Ziqi

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "main.h"
#include "bsp_usart.h"
#include "bsp_define.h"
#include "bsp_kinematics.h"

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

short MessageData[255] = {0};

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static uint8_t USART_Robot_RX_Package[255];     //接收缓冲，数据内容小于等于32Byte


//外部变量
extern int16_t motor_kp;
extern int16_t motor_ki;
extern int16_t motor_kd;

int16_t robot_target_speed[3] = {0};  // X Y Yaw
int16_t robot_params[2] = {0};

u8 USART_Debug_RX_Buffer[1];//HAL库使用的串口接收缓冲
u8 USART_Robot_RX_Buffer[1];//HAL库使用的串口接收缓冲
static uint32_t USART_Robot_RX_Count=0;       //接收计数器

int fputc(int c, FILE *stream)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);   
    return 1;
}

uint8_t ch_r;
int fgetc(FILE * F)    
{
    HAL_UART_Receive(&huart1,&ch_r,1,0xffff);//接收
    return ch_r;
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
		HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // 重新开启接收中断
	}
	
	
	if (huart->Instance==USART2)
	{
		if(USART_Robot_RX_Count < 2) //==接收帧头 + 长度
		{
			if(USART_Robot_RX_Count == 0) //接收帧头1 0xAA
			{
				if(USART_Robot_RX_Buffer[0] == 0x55)
				{
					USART_Robot_RX_Package[0] = USART_Robot_RX_Buffer[0];
					USART_Robot_RX_Count = 1;
				}
				else
				{
					USART_Robot_RX_Count = 0;
				}
			}
			else if(USART_Robot_RX_Count == 1) //接收帧头2 0x55
			{
				if(USART_Robot_RX_Buffer[0] == 0xAA)
				{
					USART_Robot_RX_Package[1] = USART_Robot_RX_Buffer[0];
					USART_Robot_RX_Count = 2;
				}
				else
				{
					USART_Robot_RX_Count = 0;
				}				
			}
		}
		else if(USART_Robot_RX_Count == 2) //接收数据长度
		{
			USART_Robot_RX_Package[2] = USART_Robot_RX_Buffer[0];
			USART_Robot_RX_Count = 3;
		}
		else if(USART_Robot_RX_Count >= 3) //接收数据
		{
			if(USART_Robot_RX_Count < ((unsigned int)USART_Robot_RX_Package[2]-3) )
			{
				USART_Robot_RX_Package[USART_Robot_RX_Count] = USART_Robot_RX_Buffer[0];
				USART_Robot_RX_Count++;			
			}
			else if(USART_Robot_RX_Count == ((unsigned int)USART_Robot_RX_Package[2]-3))
			{
				//数据校验
				if( USART_Robot_RX_Buffer[0] == CalCrc(USART_Robot_RX_Package, (unsigned int)USART_Robot_RX_Package[2]-3))  //校验正确
				{
					USART_Robot_RX_Package[USART_Robot_RX_Count] = USART_Robot_RX_Buffer[0];
					USART_Robot_RX_Count++;
				}
				else
				{
					USART_Robot_RX_Count = 0;
				}
			}
			else if(USART_Robot_RX_Count > ((unsigned int)USART_Robot_RX_Package[2]-3))
			{
				if(USART_Robot_RX_Count == ((unsigned int)USART_Robot_RX_Package[2]-3+1)) //接收帧头1 0xAA
				{
					if(USART_Robot_RX_Buffer[0] == 0x0D)
					{
						USART_Robot_RX_Package[USART_Robot_RX_Count] = USART_Robot_RX_Buffer[0];
						USART_Robot_RX_Count++;
					}
					else
					{
						USART_Robot_RX_Count = 0;
					}
				}
				else if(USART_Robot_RX_Count == ((unsigned int)USART_Robot_RX_Package[2]-3+2)) //接收帧头2 0x55
				{
					if(USART_Robot_RX_Buffer[0] == 0x0A)
					{
						USART_Robot_RX_Package[USART_Robot_RX_Count] = USART_Robot_RX_Buffer[0];
						USART_Robot_RX_Count++;
					}
					else
					{
						USART_Robot_RX_Count = 0;
					}				
				}
				//包完整，进行数据解析
				if(USART_Robot_RX_Count == (unsigned int)USART_Robot_RX_Package[2])
				{
					// 消息内容
					for(int i = 0; i < USART_Robot_RX_Package[2]; i++)
					{
							// 缓冲区数据送入Union
							for(int j = 0; j < 2; j++)
							{
									ReceiveUnion.data_transmit[j] = USART_Robot_RX_Package[i * 2 + 4 + j];
							}
							MessageData[i] = ReceiveUnion.data; //读取数据
					}
					//此处进行数据解析
					if(USART_Robot_RX_Package[3] == 11)
					{
						robot_target_speed[0] = MessageData[0];
						robot_target_speed[1] = MessageData[1];
						robot_target_speed[2] = MessageData[2];
						
					}
					if(USART_Robot_RX_Package[3] == 12)
					{
						motor_kp = MessageData[0];
						motor_ki = MessageData[1];
						motor_kd = MessageData[2];
					}
					if(USART_Robot_RX_Package[3] == 13)
					{
						robot_params[0] = MessageData[0];
						robot_params[1] = MessageData[1];
						Kinematics_Init(robot_params);
					}
					USART_Robot_RX_Count = 0;
				}
			}
		}
		HAL_UART_Receive_IT(&huart2,(uint8_t *)USART_Robot_RX_Buffer, 1);  // 重新开启接收中断
	} 
  
}


// 串口发送函数
bool USART_Send_Pack(UART_HandleTypeDef *huart, short * WriteData, unsigned char MessageLenth, unsigned int MessageCode)
{
	// 计算发送消息长度
  unsigned char SendLength = (unsigned int)(2 * MessageLenth) + 7;

	// 发送消息缓冲区(变长数组)
  unsigned char * SendBuf = malloc(SendLength * sizeof(unsigned char));

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
	
//	// DEBUG BEGIN
//	printf("Write:(DEC -> HEX) ");
//	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
//	{
//			SendUnion.data = WriteData[i]; //更新消息Union内容
//			printf("%d -> 0x%02x 0x%02x ",SendUnion.data, (unsigned int)SendUnion.data_transmit[0], (unsigned int)SendUnion.data_transmit[1]);
//	}
//	printf("\r\nWrite:(ALL HEX) ");
//	for(int i = 0; i < SendLength; i++)
//	{
//			printf("%02x ", (unsigned int)SendBuf[i]);
//	}
//	printf("\r\nWrite: SendLength: %d\r\n", (unsigned int)SendLength);
//	// DEBUG END

	// 串口发送消息
	HAL_UART_Transmit(huart, (uint8_t *)SendBuf, SendLength, 100);
	
	//消息Union清零
	SendUnion.data = 0; 
	
	//释放内存
	free(SendBuf);

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
	HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_Robot_RX_Buffer, 1);  // 开启接收中断
	
	LOG_I("USART Init Success\r\n");
}


