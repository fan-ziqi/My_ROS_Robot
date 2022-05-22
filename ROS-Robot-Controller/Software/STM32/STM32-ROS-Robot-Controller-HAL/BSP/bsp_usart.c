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

// ��Ϣͷ/��Ϣβ����
const unsigned char HEAD[2] = {0x55, 0xaa};
const unsigned char END[2] = {0x0d, 0x0a};

// ���ڷ���/����������, data_transmit��С��data�������͵��ֽ�������
union SerialMessageUnion
{
	short data;
	unsigned char data_transmit[2];
}SendUnion, ReceiveUnion;

short MessageData[255] = {0};

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

static uint8_t USART_Robot_RX_Package[255];     //���ջ��壬��������С�ڵ���32Byte


//�ⲿ����
extern int16_t motor_kp;
extern int16_t motor_ki;
extern int16_t motor_kd;

int16_t robot_target_speed[3] = {0};  // X Y Yaw
int16_t robot_params[2] = {0};

u8 USART_Debug_RX_Buffer[1];//HAL��ʹ�õĴ��ڽ��ջ���
u8 USART_Robot_RX_Buffer[1];//HAL��ʹ�õĴ��ڽ��ջ���
static uint32_t USART_Robot_RX_Count=0;       //���ռ�����

int fputc(int c, FILE *stream)
{
    HAL_UART_Transmit(&huart1, (unsigned char *)&c, 1, 1000);   
    return 1;
}

uint8_t ch_r;
int fgetc(FILE * F)    
{
    HAL_UART_Receive(&huart1,&ch_r,1,0xffff);//����
    return ch_r;
}

extern char CMD_Buffer[];
extern int CMD_Buffer_Count;
extern int CMD_Flag;

// ��Ϣ��ʽ: 
// ��Ϣͷ1 ��Ϣͷ2 ��Ϣ���� ��Ϣ�� [��Ϣ����] У���� ��Ϣβ1 ��Ϣβ2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART1)
	{
		//���з�Ϊ0d0a
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
		HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // ���¿��������ж�
	}
	
	
	if (huart->Instance==USART2)
	{
		if(USART_Robot_RX_Count < 2) //==����֡ͷ + ����
		{
			if(USART_Robot_RX_Count == 0) //����֡ͷ1 0xAA
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
			else if(USART_Robot_RX_Count == 1) //����֡ͷ2 0x55
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
		else if(USART_Robot_RX_Count == 2) //�������ݳ���
		{
			USART_Robot_RX_Package[2] = USART_Robot_RX_Buffer[0];
			USART_Robot_RX_Count = 3;
		}
		else if(USART_Robot_RX_Count >= 3) //��������
		{
			if(USART_Robot_RX_Count < ((unsigned int)USART_Robot_RX_Package[2]-3) )
			{
				USART_Robot_RX_Package[USART_Robot_RX_Count] = USART_Robot_RX_Buffer[0];
				USART_Robot_RX_Count++;			
			}
			else if(USART_Robot_RX_Count == ((unsigned int)USART_Robot_RX_Package[2]-3))
			{
				//����У��
				if( USART_Robot_RX_Buffer[0] == CalCrc(USART_Robot_RX_Package, (unsigned int)USART_Robot_RX_Package[2]-3))  //У����ȷ
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
				if(USART_Robot_RX_Count == ((unsigned int)USART_Robot_RX_Package[2]-3+1)) //����֡ͷ1 0xAA
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
				else if(USART_Robot_RX_Count == ((unsigned int)USART_Robot_RX_Package[2]-3+2)) //����֡ͷ2 0x55
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
				//���������������ݽ���
				if(USART_Robot_RX_Count == (unsigned int)USART_Robot_RX_Package[2])
				{
					// ��Ϣ����
					for(int i = 0; i < USART_Robot_RX_Package[2]; i++)
					{
							// ��������������Union
							for(int j = 0; j < 2; j++)
							{
									ReceiveUnion.data_transmit[j] = USART_Robot_RX_Package[i * 2 + 4 + j];
							}
							MessageData[i] = ReceiveUnion.data; //��ȡ����
					}
					//�˴��������ݽ���
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
		HAL_UART_Receive_IT(&huart2,(uint8_t *)USART_Robot_RX_Buffer, 1);  // ���¿��������ж�
	} 
  
}


// ���ڷ��ͺ���
bool USART_Send_Pack(UART_HandleTypeDef *huart, short * WriteData, unsigned char MessageLenth, unsigned int MessageCode)
{
	// ���㷢����Ϣ����
  unsigned char SendLength = (unsigned int)(2 * MessageLenth) + 7;

	// ������Ϣ������(�䳤����)
  unsigned char * SendBuf = malloc(SendLength * sizeof(unsigned char));

	// ��Ϣͷ SendBuf[0] SendBuf[1]
	for(int i = 0; i < 2; i++)
	{
			SendBuf[i] = HEAD[i];
	}

	// ��Ϣ���� SendBuf[2]
	SendBuf[2] =  SendLength;

	// ��Ϣ�� SendBuf[3]
	SendBuf[3] = (unsigned char)MessageCode;
	
	// ��Ϣ���� SendBuf[4] ~ SendBuf[4 + 2*MessageLenth - 1]
	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
	{
			SendUnion.data = WriteData[i]; //������ϢUnion����
			for(int j = 0; j < 2; j++)
			{
					//���뻺����
					SendBuf[i * 2 + 4 + j] = SendUnion.data_transmit[j];
			}
	}
	
	// У������� SendBuf[2*MessageLenth + 4]
	SendBuf[2*MessageLenth + 4] = CalCrc(SendBuf, 2*MessageLenth + 4); //У��MessageLenth��Ϣ+2��Ϣͷ+1��Ϣ����+1��Ϣ��=MessageLenth + 4

	// ��Ϣβ SendBuf[MessageLenth + 5] SendBuf[MessageLenth + 6]
	SendBuf[2*MessageLenth + 5] = END[0];
	SendBuf[2*MessageLenth + 6] = END[1];
	
//	// DEBUG BEGIN
//	printf("Write:(DEC -> HEX) ");
//	for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
//	{
//			SendUnion.data = WriteData[i]; //������ϢUnion����
//			printf("%d -> 0x%02x 0x%02x ",SendUnion.data, (unsigned int)SendUnion.data_transmit[0], (unsigned int)SendUnion.data_transmit[1]);
//	}
//	printf("\r\nWrite:(ALL HEX) ");
//	for(int i = 0; i < SendLength; i++)
//	{
//			printf("%02x ", (unsigned int)SendBuf[i]);
//	}
//	printf("\r\nWrite: SendLength: %d\r\n", (unsigned int)SendLength);
//	// DEBUG END

	// ���ڷ�����Ϣ
	HAL_UART_Transmit(huart, (uint8_t *)SendBuf, SendLength, 100);
	
	//��ϢUnion����
	SendUnion.data = 0; 
	
	//�ͷ��ڴ�
	free(SendBuf);

	return true;

}

// 8λѭ������У����㺯��
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
	HAL_UART_Receive_IT(&huart1, (uint8_t *)USART_Debug_RX_Buffer, 1);  // ���������ж�
	HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_Robot_RX_Buffer, 1);  // ���������ж�
	
	LOG_I("USART Init Success\r\n");
}


