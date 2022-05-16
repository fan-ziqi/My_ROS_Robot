#include "main.h"
#include "bsp_vin.h"
#include "bsp_delay.h"

#define LOG_TAG    "VIN"
#include "bsp_log.h"

uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);
float adcx_get_chx_value_average(ADC_HandleTypeDef *ADCx, uint32_t ch, uint8_t times);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

const int resistor_divider_coefficient = (1+10)/1;
volatile float voltage_vrefint_proportion;

void VIN_Init(void)
{
	//使用内部1.2v进行校准
	init_vrefint_reciprocal();
	
	//TODO: adc自校准
	
	LOG_I("VIN Init Success\r\n");
}

uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
	static ADC_ChannelConfTypeDef ADC_ChanConf = {0};
	ADC_ChanConf.Channel = ch;
	ADC_ChanConf.Rank = 1;
	ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_55CYCLES_5; //ADC_SAMPLETIME_1CYCLE_5
	if (HAL_ADC_ConfigChannel(ADCx, &ADC_ChanConf) != HAL_OK)
	{
			Error_Handler();
	}
		
	HAL_ADC_Start(ADCx);

	HAL_ADC_PollForConversion(ADCx, 10);
	return (uint16_t)HAL_ADC_GetValue(ADCx);
}

float adcx_get_chx_value_average(ADC_HandleTypeDef *ADCx, uint32_t ch, uint8_t times)
{ 		    		  			    
	float temp_val = 0;
	for(uint8_t t = 0; t < times; t++)
	{
		temp_val += adcx_get_chx_value(ADCx, ADC_CHANNEL_8);
//		delay_ms(5);
	}
	return temp_val/times;
}

/*
VREFINT 即 ADC的内部参照电压1.2V。
本程序中，首先对内部参考电压电压VREFINT进行adc采样将其作为校准值，
在init_vrefint_reciprocal中，对VREFINT电压进行100次的采样，
接着求其均值后使用VREFINT的电压值1.2V去除以该ADC采样得到的均值，
算出voltage_vrefint_proportion，其计算公式如下，设采样得到的数字值为average_adc, 积分次数times, 参考电压vrefint：
average_adc=total_dac/times
voltage_vrefint_proportion=vrefint/average_adc=(times*vrefint)/total_dac
后续ADC中采样到的电压值与voltage_vrefint_proportion相乘就可以计算出以内部参考电压做过校准的ADC值了。
*/

void init_vrefint_reciprocal(void)
{
		LOG_D("Start VREF Reciprocal\r\n");
	
    float total_adc = 0;
		const int times = 1000;
		const float vrefint = 1.2f;
    for(int i = 0; i < times; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT); // * (3.3f/4096)
    }
		LOG_D("total_adc = %f\r\n",total_adc);
		LOG_D("VREF = %f\r\n",(total_adc/1000)*(3.3/4096));
    voltage_vrefint_proportion = times * vrefint / total_adc;
		LOG_D("voltage_vrefint_proportion = %f\r\n",voltage_vrefint_proportion);
		
		LOG_D("End VREF Reciprocal\r\n");
}

/*
接着通过ADC对经过了分压电路的电池电压值进行采样，
将该采样结果与voltage_vrefint_proportion相乘，就得到了ADC采样值，
由于这个采样值是分压后的结果，需要反向计算出电压的值。
分压的电阻值为10K和1K，由于(1K+10K)/1K=11，乘以这个值之后就可以得到电池的电压值。
*/

float get_battery_voltage(void)
{
    float voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value_average(&hadc2, ADC_CHANNEL_8, 100); //100次采样取平均值
		voltage = (float)adcx * resistor_divider_coefficient * voltage_vrefint_proportion; // * (3.3f/4096) 
		return voltage;
}


/*
还可以通过ADC获得板载的温度传感器的温度值，同样是先经过ADC值进行采样，采样结束后，将ADC采样结果adc带入公式temperate = (adc - 0.76f) * 400.0f + 25.0f，从而计算出温度值。
*/
float get_temprate(void)
{
    uint16_t adcx = 0;
    float temperate;

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_TEMPSENSOR);
    temperate = (float)adcx * voltage_vrefint_proportion;
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;

    return temperate/10.0;
}
