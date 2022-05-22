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
#include "bsp_delay.h"

#define LOG_TAG    "DELAY"
#include "bsp_log.h"

static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��

//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪAHBʱ�ӣ�������������SYSTICKʱ��Ƶ��ΪAHB/8
//����Ϊ�˼���FreeRTOS�����Խ�SYSTICK��ʱ��Ƶ�ʸ�ΪAHB��Ƶ�ʣ�
//SYSCLK:ϵͳʱ��Ƶ��
void Delay_Init()
{
//	u32 reload;
//	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//ѡ���ⲿʱ��  HCLK
//	fac_us=SystemCoreClock/1000000;				//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
//	reload=SystemCoreClock/1000000;				//ÿ���ӵļ������� ��λΪM  
//	reload*=1000000/configTICK_RATE_HZ;			//����configTICK_RATE_HZ�趨���ʱ��
//												//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��0.233s����	
//	fac_ms=1000/configTICK_RATE_HZ;				//����OS������ʱ�����ٵ�λ	   
//	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
//	SysTick->LOAD=reload; 						//ÿ1/configTICK_RATE_HZ���ж�һ��	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK  

	fac_us = SystemCoreClock / 1000000;
	fac_ms = SystemCoreClock / 1000;
  
	LOG_I("DELAY Init Success\r\n");
}	

//��ʱnus
//nus:Ҫ��ʱ��us��.	
//nus:0~204522252(���ֵ��2^32/fac_us@fac_us=168)	    								   
void delay_us(u32 nus)
{		
	uint32_t ticks = 0;
	uint32_t told = 0;
	uint32_t tnow = 0;
	uint32_t tcnt = 0;
	uint32_t reload = 0;
	reload = SysTick->LOAD; //LOAD��ֵ	   				 	 
	ticks = nus * fac_us; //��Ҫ�Ľ����� 
	told = SysTick->VAL; //�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow = SysTick->VAL;	
		if(tnow != told)
		{	    
			if(tnow < told) //����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			{
				tcnt += told - tnow;
			}	
			else 
			{
				tcnt += reload - tnow + told;
			}	    
			told = tnow;
			if(tcnt >= ticks) //ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
			{
				break;
			}
		}
	}
}

void delay_ms(u32 nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

////��ʱnms
////nms:Ҫ��ʱ��ms��
////nms:0~65535
//void delay_ms(u32 nms)
//{	
//	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
//	{		
//		if(nms>=fac_ms)						//��ʱ��ʱ�����OS������ʱ������ 
//		{ 
//   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS��ʱ
//		}
//		nms%=fac_ms;						//OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
//	}
//	delay_us((u32)(nms*1000));				//��ͨ��ʽ��ʱ
//}

//��ʱnms,���������������
//nms:Ҫ��ʱ��ms��
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}
