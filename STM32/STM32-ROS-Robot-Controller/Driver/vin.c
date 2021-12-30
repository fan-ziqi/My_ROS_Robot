#include "vin.h"
#include "delay.h"


float PIANYI=0.15;

/**
  * @简  述  VIN 输入电压检测初始化
  * @参  数  无
  * @返回值  无
  */
void VIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//配置GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//**配置ADC******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	//ADC模式配置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);	

	//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//配置ADC通道、转换顺序和采样时间
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);

	//使能ADC
	ADC_Cmd(ADC2, ENABLE);

	//初始化ADC 校准寄存器  
	ADC_ResetCalibration(ADC2);

	//等待校准寄存器初始化完成
	while(ADC_GetResetCalibrationStatus(ADC2));

	//ADC开始校准
	ADC_StartCalibration(ADC2);

	//等待校准完成
	while(ADC_GetCalibrationStatus(ADC2)); 	
}

u16  Get_adc2(u8 ch)
{   
			//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC2);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_adc2_Average(u8 ch,u8 times)
{ 		    		  			    
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_adc2(ch);
		delay_ms(5);
	}
	return temp_val/times;
	
}

float Get_VCC(void)
{
	u16 adcx;
	float vcc;
	
	adcx = Get_adc2_Average(ADC_Channel_8,10);  //获取adc的值
	vcc=(float)adcx*(3.3*11/4096)+PIANYI;     				//求当前电压
	return vcc;
}
