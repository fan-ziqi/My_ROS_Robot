#include "main.h"
#include "bsp_iic.h"
#include "bsp_delay.h"

#define LOG_TAG    "IIC"
#include "bsp_log.h"

//MPU IIC 延时函数
void IIC_Delay(void)
{
	for(int i=0;i<1;i++) ;
//	osDelay(1);
}

//初始化IIC
void IIC_Init(void)
{	
	//此处的初始化函数在gpio.c中实现，通过CubeMX配置。
//    GPIO_InitTypeDef GPIO_Initure;

//    __HAL_RCC_GPIOB_CLK_ENABLE();           	//开启GPIOB时钟
//	
//    GPIO_Initure.Pin=SCL_Pin|SDA_Pin; 	//PB10、11
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//推挽输出
//    GPIO_Initure.Pull=GPIO_PULLUP;          	//上拉
//    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
//    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	IIC_SCL=1;
	IIC_SDA=1;
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	 
	IIC_SCL=1;
	IIC_Delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	

//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_Delay();
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	IIC_Delay();							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;IIC_Delay();	   
	IIC_SCL=1;IIC_Delay();	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	IIC_Delay();
	IIC_SCL=1;
	IIC_Delay();
	IIC_SCL=0;
}

//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	IIC_Delay();
	IIC_SCL=1;
	IIC_Delay();
	IIC_SCL=0;
}					 				     

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	SDA_OUT(); 	    
	IIC_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		IIC_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		IIC_SCL=1;
		IIC_Delay(); 
		IIC_SCL=0;	
		IIC_Delay();
	}	 
} 	    

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		IIC_SCL=0; 
		IIC_Delay();
		IIC_SCL=1;
		receive<<=1;
		if(READ_SDA)receive++;   
		IIC_Delay(); 
  }					 
	if (!ack)
			IIC_NAck();//发送nACK
	else
			IIC_Ack(); //发送ACK   
	return receive;
}


