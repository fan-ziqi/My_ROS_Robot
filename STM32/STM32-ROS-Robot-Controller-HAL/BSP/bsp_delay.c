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

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

//初始化延迟函数
//SYSTICK的时钟固定为AHB时钟，基础例程里面SYSTICK时钟频率为AHB/8
//这里为了兼容FreeRTOS，所以将SYSTICK的时钟频率改为AHB的频率！
//SYSCLK:系统时钟频率
void Delay_Init()
{
//	u32 reload;
//	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//选择外部时钟  HCLK
//	fac_us=SystemCoreClock/1000000;				//不论是否使用OS,fac_us都需要使用
//	reload=SystemCoreClock/1000000;				//每秒钟的计数次数 单位为M  
//	reload*=1000000/configTICK_RATE_HZ;			//根据configTICK_RATE_HZ设定溢出时间
//												//reload为24位寄存器,最大值:16777216,在72M下,约合0.233s左右	
//	fac_ms=1000/configTICK_RATE_HZ;				//代表OS可以延时的最少单位	   
//	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
//	SysTick->LOAD=reload; 						//每1/configTICK_RATE_HZ秒中断一次	
//	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK  

	fac_us = SystemCoreClock / 1000000;
	fac_ms = SystemCoreClock / 1000;
  
	LOG_I("DELAY Init Success\r\n");
}	

//延时nus
//nus:要延时的us数.	
//nus:0~204522252(最大值即2^32/fac_us@fac_us=168)	    								   
void delay_us(u32 nus)
{		
	uint32_t ticks = 0;
	uint32_t told = 0;
	uint32_t tnow = 0;
	uint32_t tcnt = 0;
	uint32_t reload = 0;
	reload = SysTick->LOAD; //LOAD的值	   				 	 
	ticks = nus * fac_us; //需要的节拍数 
	told = SysTick->VAL; //刚进入时的计数器值
	while(1)
	{
		tnow = SysTick->VAL;	
		if(tnow != told)
		{	    
			if(tnow < told) //这里注意一下SYSTICK是一个递减的计数器就可以了.
			{
				tcnt += told - tnow;
			}	
			else 
			{
				tcnt += reload - tnow + told;
			}	    
			told = tnow;
			if(tcnt >= ticks) //时间超过/等于要延迟的时间,则退出.
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

////延时nms
////nms:要延时的ms数
////nms:0~65535
//void delay_ms(u32 nms)
//{	
//	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
//	{		
//		if(nms>=fac_ms)						//延时的时间大于OS的最少时间周期 
//		{ 
//   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS延时
//		}
//		nms%=fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
//	}
//	delay_us((u32)(nms*1000));				//普通方式延时
//}

//延时nms,不会引起任务调度
//nms:要延时的ms数
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}
