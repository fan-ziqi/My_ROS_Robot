#include "motor.h" 

/**
  * @简  述  电机PWM控制初始化	
  * @参  数  freq_khz:PWM输出频率，范围1~20,单位KHz
  * @返回值  无
  */
void MOTOR_Init(uint8_t freq_khz)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; 

	//定时器通道IO配置
	//GPIO及复用功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//电机方向控制IO配置
	//IO时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	//电机A方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//电机B方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//电机C方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//电机D方向控制IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);



	//定时器配置
	//参数过滤
	if(freq_khz == 0)
	freq_khz = 1;
	if(freq_khz > 20)
	freq_khz = 20;
	 
	//TIM时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	//Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 2000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 36/freq_khz-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	//PWM1 Mode configuration: Channel1 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel3
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	//PWM1 Mode configuration: Channel4
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);

	//TIM enable counter
	TIM_Cmd(TIM8, ENABLE);   

	//使能MOE位
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	
}

/**
  * @简  述 电机1 PWM速度控制
  * @参  数 speed 电机转速数值，范围-2000~2000
  * @返回值 无
  */
void MOTOR1_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
		GPIO_SetBits(GPIOC, GPIO_Pin_0);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_1);
		GPIO_ResetBits(GPIOC, GPIO_Pin_0);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
		GPIO_ResetBits(GPIOC, GPIO_Pin_0);
		temp = 0;
	}
	
	if(temp>2000) temp = 2000;
	
	TIM_SetCompare1(TIM8,temp);
}


/**
  * @简  述 电机2 PWM速度控制
  * @参  数 speed 电机转速数值，范围-2000~2000
  * @返回值 无
  */
void MOTOR2_SetSpeed(int16_t speed)
{
	uint16_t temp;

    if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
		GPIO_SetBits(GPIOC, GPIO_Pin_3);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_2);
		GPIO_ResetBits(GPIOC, GPIO_Pin_3);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_2);
		GPIO_ResetBits(GPIOC, GPIO_Pin_3);
		temp = 0;
	}
	
	if(temp>2000) temp = 2000;
	
	TIM_SetCompare2(TIM8,temp);
}
/**
  * @简  述 电机3 PWM速度控制
  * @参  数 speed 电机转速数值，范围-2000~2000
  * @返回值 无
  */
void MOTOR3_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_5);
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
		temp = 0;
	}
	
	if(temp>2000) temp = 2000;
	
	TIM_SetCompare3(TIM8,temp);
}

/**
  * @简  述 电机4 PWM速度控制
  * @参  数 speed 电机转速数值，范围-2000~2000
  * @返回值 无
  */
void MOTOR4_SetSpeed(int16_t speed)
{
	uint16_t temp;

	if(speed > 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		temp = speed;	
	}
	else if(speed < 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		temp = (-speed);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		temp = 0;
	}
	
	if(temp>2000) temp = 2000;
	
	TIM_SetCompare4(TIM8,temp);
}

