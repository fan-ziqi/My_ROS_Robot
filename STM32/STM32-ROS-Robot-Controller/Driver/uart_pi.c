/**			                                                    
  ******************************************************************************
  * @说  明
  *
  * 1.树莓派通信串口
  * 2.可以使用具有X-Protocol协议进行数据发送
  * 3.开启UART的串口数据接收功能，使用中断方式，X-Protocol协议通信
	* 4.可通过UART_XX_GetRxData()函数判断是否有数据接收
  *
  * X-Protocol协议介绍（变帧长）
  * 帧定义：AA 55 | 0B  | 01  | 03 E8 FC 18 00 0A | 14
  *        帧头   帧长   帧码  数据                校验和
  * 帧  头：双帧头，抗干扰强
  * 帧  长：根据数据长度设定
  * 帧  码：用户根据功能设定，标识帧的唯一性
  * 数  据：高位在前，长度可变，内容自由组合8位，16位，32位数据
  * 校验和：前面数据累加和的低8位
  * 帧示例：( AA 55 0B 01 03 E8 FC 18 00 0A 14 ) 内容：1000，-1000,10,
  ******************************************************************************
  */

#include "uart_pi.h"
#include <stdio.h>
#include "kinematics.h"

static uint8_t uart2_flag_rx_ok = 0; //接收成功标志
static uint8_t uart2_rx_con=0;  //接收计数器
static uint8_t uart2_rx_checksum; //帧头部分校验和
static uint8_t uart2_rx_buf[60];  //接收缓冲，数据内容小于等于32Byte
static uint8_t uart2_tx_buf[60];  //发送缓冲

//外部变量
extern int16_t motor_kp;
extern int16_t motor_ki;
extern int16_t motor_kd;

extern int16_t robot_target_speed[3];  // X Y Yaw
extern int16_t robot_params[2];


/**
  * @简  述  串口初始化
  * @参  数  baud： 波特率设置
  * @返回值	 无
  */
void UART_PI_Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//**USART配置******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //打开串口GPIO的时钟

	//将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //打开串口外设的时钟

	//配置USART参数
	USART_InitStructure.USART_BaudRate = baud;		//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	//配置USART为中断源
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢断优先级	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	//使能中断
	NVIC_Init(&NVIC_InitStructure);//初始化配置NVIC

	//使能串口接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	//使能 USART， 配置完毕
	USART_Cmd(USART2, ENABLE);
}



/**
  * @简  述  串口中断服务程序
  * @参  数  无
  * @返回值  无
  */
void USART2_IRQHandler(void)                	
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		  //printf("Get Data!\r\n");
			Res =USART_ReceiveData(USART2);	
		
			if(uart2_rx_con < 3)    //==接收帧头 + 长度
			{
				if(uart2_rx_con == 0)  //接收帧头1 0xAA
				{
					if(Res == 0xAA)
					{
						uart2_rx_buf[0] = Res;
						uart2_rx_con = 1;					
					}
					else
					{
						
					}
				}else if(uart2_rx_con == 1) //接收帧头2 0x55
				{
					if(Res == 0x55)
					{
						uart2_rx_buf[1] = Res;
						uart2_rx_con = 2;
					}
					else
					{
						uart2_rx_con = 0;						
					}				
				}
				else  //接收数据长度
				{
					uart2_rx_buf[2] = Res;
					uart2_rx_con = 3;
					uart2_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
				}
			}
			else    //==接收数据
			{
				if(uart2_rx_con < (uart2_rx_buf[2]-1) )
				{
					uart2_rx_buf[uart2_rx_con] = Res;
					uart2_rx_con++;
					uart2_rx_checksum = uart2_rx_checksum + Res;					
				}
				else    //判断最后1位
				{
					//数据校验
					if( Res == uart2_rx_checksum )  //校验正确
					{	
						//=====此处进行数据解析=========
						//运动控制帧
						if(uart2_rx_buf[3] == 0x11)
						{
							robot_target_speed[0] = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
							robot_target_speed[1] = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
							robot_target_speed[2] = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
						  
							//速度限制
							if(robot_target_speed[0] > ROBOT_LINEAR_SPEED_LIMIT)    robot_target_speed[0] = ROBOT_LINEAR_SPEED_LIMIT;
							if(robot_target_speed[0] < (-ROBOT_LINEAR_SPEED_LIMIT)) robot_target_speed[0] = (-ROBOT_LINEAR_SPEED_LIMIT);
							if(robot_target_speed[1] > ROBOT_LINEAR_SPEED_LIMIT)    robot_target_speed[1] = ROBOT_LINEAR_SPEED_LIMIT;
							if(robot_target_speed[1] < (-ROBOT_LINEAR_SPEED_LIMIT)) robot_target_speed[1] = (-ROBOT_LINEAR_SPEED_LIMIT);
							if(robot_target_speed[2] > ROBOT_ANGULAR_SPEED_LIMIT)    robot_target_speed[2] = ROBOT_ANGULAR_SPEED_LIMIT;
							if(robot_target_speed[2] < (-ROBOT_ANGULAR_SPEED_LIMIT)) robot_target_speed[2] = (-ROBOT_ANGULAR_SPEED_LIMIT);
						}
						else
						{
							//PID参数帧
							if(uart2_rx_buf[3] == 0x12)
							{
								motor_kp = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
								motor_ki = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
								motor_kd = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
							}
							
							//机器人参数
							if(uart2_rx_buf[3] == 0x13)
							{
								robot_params[0] = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
								robot_params[1] = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
								
								Kinematics_Init(robot_params);
							}
						}
						
						//接收完成，恢复初始状态
						uart2_rx_con = 0;					
					}	
					
				}
			}
			
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	} 
}

/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：接收数据指针,第1个字节为帧编码，后面为数据
  * @返回值	 0-无数据接收，other-需要读取的数据字节个数
  */
uint8_t UART_PI_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart2_flag_rx_ok != 0)
	{
		cnt = uart2_rx_buf[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = uart2_rx_buf[3+i];
		}
		
		uart2_flag_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

/**
  * @简  述  发送数据（X-Protocol协议）
  * @参  数  *pbuf：发送数据指针
  *          len：发送数据长度个数，≤27 (32-5)
  *          num：帧号，帧编码
  * @返回值	 无
  */
void UART_PI_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
  uint8_t tx_checksum = 0;//发送校验和
	
	if(len <= 50)
	{
		/******获取数据******/
		uart2_tx_buf[0] = 0xAA;    //帧头
		uart2_tx_buf[1] = 0x55;    //
		uart2_tx_buf[2] = len+5;  //根据输出长度计算帧长度
		uart2_tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			uart2_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******计算校验和******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart2_tx_buf[i];
		}
		uart2_tx_buf[i] = tx_checksum;
		
		
		/******发送数据******/	
		cnt = 5+len;
		
		//查询传输方式
		for(i=0; i<cnt; i++)
		{
			USART_SendData(USART2, uart2_tx_buf[i]);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);
		}
	}
}

