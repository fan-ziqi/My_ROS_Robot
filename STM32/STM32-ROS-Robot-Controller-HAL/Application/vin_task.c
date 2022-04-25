#include "bsp_vin.h"
#include "bsp_led.h"

#define LOG_TAG    "VIN"
#include "bsp_log.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;


int battery_status = IS_FULL;

#define VOLTAGE_DROP            0.312712f
#define VOLTAGE_CHG							11.50f
#define VOLTAGE_OFF							11.00f

float battery_voltage;

//100ms
void vin_task(void const * argument)
{
		osDelay(1000);
		//use inner 1.2v to calbrate
    init_vrefint_reciprocal();
		LOG_I("VIN Init Success\r\n");
	
    while(1)
    {
        battery_voltage = get_battery_voltage() + VOLTAGE_DROP;
			
//				LOG_D("��ǰ��ѹ = %f\r\n", battery_voltage);
//				LOG_D("��ǰ�¶� = %f\r\n", get_temprate());
			
				if(battery_voltage < VOLTAGE_CHG)
				{
					//��Ҫ���
					battery_status = NEED_CHARGE;
				  
					if(battery_voltage < VOLTAGE_OFF)
					{
						osDelay(5000);

						//��ѹ����С��VOLTAGE_OFF 5�룬����ͣ������״̬
						if(battery_voltage < VOLTAGE_OFF)
						{
							battery_status = NEED_POWEROFF;
							//ֹͣ������ڵ����������
							
							//ֹͣ���
							while(1)
							{
								osDelay(100);
							}								
						}
					}
				}
				else
				{
					//�������
					battery_status = IS_FULL;
				}
				
        osDelay(100);
    }
}
