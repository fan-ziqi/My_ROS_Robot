#include "vin.h"
#include "delay.h"


float PIANYI=0.15;

/**
  * @��  ��  VIN �����ѹ����ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void VIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//����GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//**����ADC******
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	//ADCģʽ����
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);	

	//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	//����ADCͨ����ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);

	//ʹ��ADC
	ADC_Cmd(ADC2, ENABLE);

	//��ʼ��ADC У׼�Ĵ���  
	ADC_ResetCalibration(ADC2);

	//�ȴ�У׼�Ĵ�����ʼ�����
	while(ADC_GetResetCalibrationStatus(ADC2));

	//ADC��ʼУ׼
	ADC_StartCalibration(ADC2);

	//�ȴ�У׼���
	while(ADC_GetCalibrationStatus(ADC2)); 	
}

u16  Get_adc2(u8 ch)
{   
			//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC2);	//�������һ��ADC1�������ת�����
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
	
	adcx = Get_adc2_Average(ADC_Channel_8,10);  //��ȡadc��ֵ
	vcc=(float)adcx*(3.3*11/4096)+PIANYI;     				//��ǰ��ѹ
	return vcc;
}
