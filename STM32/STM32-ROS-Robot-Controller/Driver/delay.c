#include "delay.h"

/**
  * @��  ��  ��ʱ������ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void DELAY_Init(void) 
{	
	//��ʱ����SysTick����
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 	//ʱ�ӹ̶�ΪAHBʱ�ӵ�1/8
}

/**
  * @��  ��  ���΢����ʱ
  * @��  ��  us����ʱ���ȣ���λus	  
  * @����ֵ  ��
  */
void Delayus(uint16_t us)
{
	uint32_t temp;
	
	SysTick->LOAD=9*us; 				 		 
	SysTick->VAL=0x00;        				
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  	 
	
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));	 
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; 
	SysTick->VAL =0X00;       				
}
/**
  * @��  ��  ��������ʱ������
  * @��  ��  ms����ʱ���ȣ���λms	  	  
  * @����ֵ  ��
  * @˵  ��  ע��ms�ķ�Χ��SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:nTime<=0xffffff*8*1000/SYSCLK
  *          ��72M������,ms<=1864ms 
  */
static void Delay_ms(uint16_t ms)
{	 		  	  
	uint32_t temp;	
	
	SysTick->LOAD=(uint32_t)9000*ms;
	SysTick->VAL =0x00;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	
	
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;      
	SysTick->VAL =0X00;     		  		 	    
}

/**
  * @��  ��  ���������ʱ
  * @��  ��  ms����ʱ���ȣ���λms	  	 	  
  * @����ֵ  ��
  */
void delay_ms(uint16_t ms)
{
	uint8_t repeat=ms/500;																
	uint16_t remain=ms%500;
	
	while(repeat)
	{
		Delay_ms(500);
		repeat--;
	}
	
	if(remain)
	{
		Delay_ms(remain);
	}
}
