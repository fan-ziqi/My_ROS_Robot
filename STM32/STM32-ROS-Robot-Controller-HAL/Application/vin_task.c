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
			
//				LOG_D("当前电压 = %f\r\n", battery_voltage);
//				LOG_D("当前温度 = %f\r\n", get_temprate());
			
				if(battery_voltage < VOLTAGE_CHG)
				{
					//需要充电
					battery_status = NEED_CHARGE;
				  
					if(battery_voltage < VOLTAGE_OFF)
					{
						osDelay(5000);

						//电压持续小于VOLTAGE_OFF 5秒，进入停机保护状态
						if(battery_voltage < VOLTAGE_OFF)
						{
							battery_status = NEED_POWEROFF;
							//停止电机放在电机的任务里
							
							//停止检测
							while(1)
							{
								osDelay(100);
							}								
						}
					}
				}
				else
				{
					//电池正常
					battery_status = IS_FULL;
				}
				
        osDelay(100);
    }
}
