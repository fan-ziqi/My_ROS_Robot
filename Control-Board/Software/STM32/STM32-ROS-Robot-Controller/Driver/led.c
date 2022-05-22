#include "led.h"

/**
  * @��  ��  LED ��ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void LED_Init(void) 
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//LED GPIO����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	//�ر�LED��
	GPIO_SetBits(GPIOD,GPIO_Pin_2);
	GPIO_SetBits(GPIOC,GPIO_Pin_12);
}


