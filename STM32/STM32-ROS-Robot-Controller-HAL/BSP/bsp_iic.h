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

#ifndef BSP_IIC_H
#define BSP_IIC_H
#include "bsp_define.h"

//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;}

//IO��������	 
#define IIC_SCL    PBout(8) 		//SCL
#define IIC_SDA    PBout(9) 		//SDA	 
#define READ_SDA   PBin(9) 	  	//����SDA 

//IIC���в�������
void IIC_Delay(void);				//MPU IIC��ʱ����
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  



#endif
