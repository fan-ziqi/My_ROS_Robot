#include "servo.h" 
/**
  * @��  ��  ���AB �ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void SERVO_AB_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 	
	
	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  
	//ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;	    //ռ�ձȳ�ʼ����90��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM2, ENABLE);	
}

/**
  * @��  ��  ���A����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_A_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare1(TIM2,angle);
}

/**
  * @��  ��  ���B����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_B_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare2(TIM2,angle);
}

/**
  * @��  ��  ���CD �ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void SERVO_CD_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3 , ENABLE); //���������ӳ�书�ܺ���

	//��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  
	//ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;	    //ռ�ձȳ�ʼ����90��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM3, ENABLE);	
}

/**
  * @��  ��  ���E����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_C_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare1(TIM3,angle);
}

/**
  * @��  ��  ���E����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_D_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare2(TIM3,angle);
}


/**
  * @��  ��  ���EF �ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void SERVO_EF_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  
	//ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;	    //ռ�ձȳ�ʼ����90��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM4, ENABLE);	
}

/**
  * @��  ��  ���E����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_E_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare1(TIM4,angle);
}

/**
  * @��  ��  ���F����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_F_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare2(TIM4,angle);
}

/**
  * @��  ��  ���GH �ӿڳ�ʼ��	
  * @��  ��  ��
  * @����ֵ  ��
  */
void SERVO_GH_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//��ʱ������	PWM����ģʽ��Ƶ��50Hz,����20ms  
	//ռ�ձȵ��ڷ�Χ��0-1.5ms-2.5ms 0-1500-2500	��ʼ��Ϊ1500
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;  //��ʱ����Ƶ����Ƶ���Ƶ��Ϊ1M
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=20000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500;	    //ռ�ձȳ�ʼ����90��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM5, ENABLE);

	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM5, ENABLE);	
}

/**
  * @��  ��  ���G����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_G_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare1(TIM5,angle);
}

/**
  * @��  ��  ���H����
  * @��  ��  angle ����ĽǶȣ���Χ��0~1800������ϵ��0.1,
             ˵�������ֶ��ʵ�ʿ��ƽǶ�С��0~1800����ע�ⷶΧ����
  * @����ֵ  ��
  */
void SERVO_H_SetAngle(uint16_t angle)
{
	if(angle > 1800)
		angle = 1800;
	
	angle = 1.111f*angle + 500;
	TIM_SetCompare2(TIM5,angle);
}

/******************* (C) ��Ȩ 2018 XTARK **************************************/
