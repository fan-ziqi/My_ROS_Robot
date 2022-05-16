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
	//ʹ���ڲ�1.2v����У׼
	init_vrefint_reciprocal();
	
	//TODO: adc��У׼
	
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
VREFINT �� ADC���ڲ����յ�ѹ1.2V��
�������У����ȶ��ڲ��ο���ѹ��ѹVREFINT����adc����������ΪУ׼ֵ��
��init_vrefint_reciprocal�У���VREFINT��ѹ����100�εĲ�����
���������ֵ��ʹ��VREFINT�ĵ�ѹֵ1.2Vȥ���Ը�ADC�����õ��ľ�ֵ��
���voltage_vrefint_proportion������㹫ʽ���£�������õ�������ֵΪaverage_adc, ���ִ���times, �ο���ѹvrefint��
average_adc=total_dac/times
voltage_vrefint_proportion=vrefint/average_adc=(times*vrefint)/total_dac
����ADC�в������ĵ�ѹֵ��voltage_vrefint_proportion��˾Ϳ��Լ�������ڲ��ο���ѹ����У׼��ADCֵ�ˡ�
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
����ͨ��ADC�Ծ����˷�ѹ��·�ĵ�ص�ѹֵ���в�����
���ò��������voltage_vrefint_proportion��ˣ��͵õ���ADC����ֵ��
�����������ֵ�Ƿ�ѹ��Ľ������Ҫ����������ѹ��ֵ��
��ѹ�ĵ���ֵΪ10K��1K������(1K+10K)/1K=11���������ֵ֮��Ϳ��Եõ���صĵ�ѹֵ��
*/

float get_battery_voltage(void)
{
    float voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value_average(&hadc2, ADC_CHANNEL_8, 100); //100�β���ȡƽ��ֵ
		voltage = (float)adcx * resistor_divider_coefficient * voltage_vrefint_proportion; // * (3.3f/4096) 
		return voltage;
}


/*
������ͨ��ADC��ð��ص��¶ȴ��������¶�ֵ��ͬ�����Ⱦ���ADCֵ���в��������������󣬽�ADC�������adc���빫ʽtemperate = (adc - 0.76f) * 400.0f + 25.0f���Ӷ�������¶�ֵ��
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
