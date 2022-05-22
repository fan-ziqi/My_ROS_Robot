#include "encoder.h" 

/**
  * @简  述  编码器Motor1初始化
  * @参  数  cycle：计数周期
  * @返回值  无
  */
void ENCODER_Motor1_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	//Reset counter
	TIM2->CNT = 0;

	TIM_Cmd(TIM2, ENABLE);   
}

/**
  * @简  述  编码器Motor2获取计数器数值
  * @参  数  无
  * @返回值  计数器当前值
  */
uint16_t ENCODER_Motor1_GetCounter(void)
{
	return (TIM_GetCounter(TIM2)); 
}

/**
  * @简  述  编码器Motor1设置计数器数值
  * @参  数  count  计数器数值
  * @返回值  无
  */
void ENCODER_Motor1_SetCounter(uint16_t count)
{
	TIM2->CNT = count;
}

/**
  * @简  述  编码器Motor2初始化
  * @参  数  cycle：计数周期
  * @返回值  无
  */
void ENCODER_Motor2_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3 , ENABLE); //这个就是重映射功能函数

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//Reset counter
	TIM3->CNT = 0;

	TIM_Cmd(TIM3, ENABLE);  
}

/**
  * @简  述  编码器Motor2获取计数器数值
  * @参  数  无
  * @返回值  计数器当前值
  */
uint16_t ENCODER_Motor2_GetCounter(void)
{
	return (TIM_GetCounter(TIM3)); 
}

/**
  * @简  述  编码器Motor2设置计数器数值
  * @参  数  count  计数器数值
  * @返回值  无
  */
void ENCODER_Motor2_SetCounter(uint16_t count)
{
	TIM3->CNT = count;
}

/**
  * @简  述  编码器Motor3初始化
  * @参  数  cycle：计数周期
  * @返回值  无
  */
void ENCODER_Motor3_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	//Reset counter
	TIM4->CNT = 0;

	TIM_Cmd(TIM4, ENABLE);     
}

/**
  * @简  述  编码器Motor3获取计数器数值
  * @参  数  无
  * @返回值  计数器当前值
  */
uint16_t ENCODER_Motor3_GetCounter(void)
{
	return (TIM_GetCounter(TIM4)); 
}

/**
  * @简  述  编码器Motor3设置计数器数值
  * @参  数  count  计数器数值
  * @返回值  无
  */
void ENCODER_Motor3_SetCounter(uint16_t count)
{
	TIM4->CNT = count;
}

/**
  * @简  述  编码器Motor4初始化
  * @参  数  cycle：计数周期
  * @返回值  无
  */
void ENCODER_Motor4_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	//GPIO功能时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIM时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	//Reset counter
	TIM5->CNT = 0;

	TIM_Cmd(TIM5, ENABLE);    
}

/**
  * @简  述  编码器Motor4获取计数器数值
  * @参  数  无
  * @返回值  计数器当前值
  */
uint16_t ENCODER_Motor4_GetCounter(void)
{
	return (TIM_GetCounter(TIM5)); 
}

/**
  * @简  述  编码器GH设置计数器数值
  * @参  数  count  计数器数值
  * @返回值  无
  */
void ENCODER_Motor4_SetCounter(uint16_t count)
{
	TIM5->CNT = count;
}
