#include "key.h"
#include "delay.h"
#include "sys.h"

int key1_state=0,key2_state=0;

/**
  * @��  ��  KEY ������ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//ʹ����GPIO
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE );
    PWR_BackupAccessCmd( ENABLE );/* �����޸�RTC�ͺ󱸼Ĵ���*/
    RCC_LSEConfig( RCC_LSE_OFF ); /* �ر��ⲿ����ʱ��,PC14+PC15����������ͨIO*/
    BKP_TamperPinCmd(DISABLE);  /* �ر����ּ�⹦��,PC13����������ͨIO*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	PWR_BackupAccessCmd(DISABLE);/* ��ֹ�޸�RTC�ͺ󱸼Ĵ���*/
    //BKP_ITConfig(DISABLE);       /* ��ֹTAMPER �ж�*/
	
}



/**
  * @��  ��  KEY ��ȡ����ֵ
  * @��  ��  ��
  * @����ֵ  ����ֵ����������Ϊ1��̧��Ϊ0
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

