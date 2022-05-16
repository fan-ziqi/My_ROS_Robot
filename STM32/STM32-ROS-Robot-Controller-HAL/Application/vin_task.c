#include "bsp_vin.h"
#include "bsp_led.h"

#define LOG_TAG    "VIN"
#include "bsp_log.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


int battery_status = IS_FULL;

#define VOLTAGE_DROP            0.13214f
#define VOLTAGE_CHG							11.50f
#define VOLTAGE_OFF							11.00f

float battery_voltage;

//100ms
void vin_task(void const * argument)
{
		osDelay(1000);
	
		VIN_Init();
	
    while(1)
    {
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
			
				if(battery_voltage < VOLTAGE_CHG)
				{
					//��Ҫ���
					battery_status = NEED_CHARGE;
				  
					if(battery_voltage < VOLTAGE_OFF)
					{
						osDelay(5000);
						battery_voltage = get_battery_voltage() + VOLTAGE_DROP;

						//��ѹ����С��VOLTAGE_OFF 5�룬����ͣ������״̬
						if(battery_voltage < VOLTAGE_OFF)
						{
							battery_status = NEED_POWEROFF;
							//ֹͣ������ڵ����������
							
							//ֹͣ���
							LOG_I("��ѹ���ͣ�ֹͣ����\r\n");
							while(1)
							{
								battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
								if(battery_voltage >= VOLTAGE_OFF)
								{
									LOG_I("��ѹ�ﵽҪ�����¹���\r\n");
									break;
								}
								osDelay(200);
							}								
						}
					}
				}
				else
				{
					//�������
					battery_status = IS_FULL;
				}
				
        osDelay(200);
    }
}
