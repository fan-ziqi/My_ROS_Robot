#include "sys.h"

/**
  * @��  ��  JTAGģʽ����
  * @��  ��  mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;  
						JTAG_SWD_DISABLE   0X02
						SWD_ENABLE         0X01
						JTAG_SWD_ENABLE    0X00	
  * @����ֵ  ��
  */
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //����jtagģʽ
} 


