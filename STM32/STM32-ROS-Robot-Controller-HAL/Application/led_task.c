#include "main.h"
#include "bsp_led.h"
#include "bsp_vin.h"

#define LOG_TAG    "LED"
#include "bsp_log.h"

extern int battery_status;

//20ms
void led_task(void const * argument)
{
	//������ʾ��Ϣ
	LED_Blue_On();
	LED_Green_On();
	osDelay(100);	
	LED_Blue_Off();
	LED_Green_Off();
	osDelay(300);

	//�̵Ƶ�������ʾ����
	LED_Green_On();
	while(1)
	{
		//ȫ����ʼ������ҵ������㣬�̵Ƶ���������Ϩ����ʾ����
		if(battery_status == IS_FULL)
		{
			LED_Blue_Off();
			LED_Green_On();
		}
		if(battery_status == NEED_CHARGE)
		{
			//��Ҫ��磬�̵Ƽ�������������˸
			LED_Green_On();
			LED_Blue_Toggle();
			if(battery_status == NEED_POWEROFF)
			{
				//�̵�Ϩ�����Ƶ���������ͣ��״̬�����ֹͣת��
				LED_Blue_On();
				LED_Green_Off();
				while(1) ;
			}
		}

		osDelay(20);
	}
}
