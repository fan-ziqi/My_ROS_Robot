#include "main.h"
#include "bsp_iic.h"
#include "bsp_delay.h"

#define LOG_TAG    "IIC"
#include "bsp_log.h"

//MPU IIC ��ʱ����
void IIC_Delay(void)
{
	for(int i=0;i<1;i++) ;
//	osDelay(1);
}

//��ʼ��IIC
void IIC_Init(void)
{	
	//�˴��ĳ�ʼ��������gpio.c��ʵ�֣�ͨ��CubeMX���á�
//    GPIO_InitTypeDef GPIO_Initure;

//    __HAL_RCC_GPIOB_CLK_ENABLE();           	//����GPIOBʱ��
//	
//    GPIO_Initure.Pin=SCL_Pin|SDA_Pin; 	//PB10��11
//    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//�������
//    GPIO_Initure.Pull=GPIO_PULLUP;          	//����
//    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //����
//    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

	IIC_SCL=1;
	IIC_SDA=1;
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	 
	IIC_SCL=1;
	IIC_Delay();
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay();
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_Delay();
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	IIC_Delay();							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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

//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	SDA_OUT(); 	    
	IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
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

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
			IIC_NAck();//����nACK
	else
			IIC_Ack(); //����ACK   
	return receive;
}


