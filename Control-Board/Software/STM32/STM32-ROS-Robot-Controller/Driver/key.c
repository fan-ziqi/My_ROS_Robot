#include "key.h"
#include "delay.h"
#include "sys.h"

int key1_state=0,key2_state=0;

/**
  * @简  述  KEY 按键初始化
  * @参  数  无
  * @返回值  无
  */
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//使配置GPIO
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE );
    PWR_BackupAccessCmd( ENABLE );/* 允许修改RTC和后备寄存器*/
    RCC_LSEConfig( RCC_LSE_OFF ); /* 关闭外部低速时钟,PC14+PC15可以用作普通IO*/
    BKP_TamperPinCmd(DISABLE);  /* 关闭入侵检测功能,PC13可以用作普通IO*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	PWR_BackupAccessCmd(DISABLE);/* 禁止修改RTC和后备寄存器*/
    //BKP_ITConfig(DISABLE);       /* 禁止TAMPER 中断*/
	
}



/**
  * @简  述  KEY 获取按键值
  * @参  数  无
  * @返回值  按键值，按键按下为1，抬起为0
  */
void KEY_Scan(void)
{
	if(PAin(8) == 0)		
	{
		delay_ms(10);
		
		if(PAin(8) == 0)
		{
			key1_state=KEY_DOWN;
		}
		else key1_state=KEY_UP;
	}
	else key1_state=KEY_UP;
	
	if(PCin(13) == 0)		
	{
		delay_ms(10);
		
		if(PCin(13) == 0)
		{
			key2_state=KEY_DOWN;
			delay_ms(100);
			if(PCin(13) == 1) key2_state=KEY_DOWNUP;
		}
		else key2_state=KEY_UP;
	}
	else key2_state=KEY_UP;
}

