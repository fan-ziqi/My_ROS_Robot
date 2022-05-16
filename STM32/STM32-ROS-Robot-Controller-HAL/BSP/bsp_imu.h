#ifndef BSP_IMU_H
#define BSP_IMU_H
#include "bsp_define.h"

#include <stdio.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "bsp_iic.h"

//////////////////////////////////////////////////////////////////
// MPU6050 ����
//////////////////////////////////////////////////////////////////

struct xyz_data
{
	int16_t x;
	int16_t y;
	int16_t z;
};

//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).
#define	MPU6050_ADDR    0x68    //MPU6050��ַ

//MPU6050�Ĵ�����ַ
//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs�Ĵ���,�ɶ�ȡ�汾��,�Ĵ����ֲ�δ�ᵽ
//#define MPU_PROD_ID_REG			0X0C	//prod id�Ĵ���,�ڼĴ����ֲ�δ�ᵽ
#define MPU6050_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU6050_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU6050_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define MPU6050_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define MPU6050_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU6050_CFG_REG				0X1A	//���üĴ���
#define MPU6050_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU6050_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU6050_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU6050_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU6050_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU6050_I2CSLV0_ADDR_REG	0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU6050_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU6050_I2CSLV0_CTRL_REG	0X27	//IIC�ӻ�0���ƼĴ���
#define MPU6050_I2CSLV1_ADDR_REG	0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU6050_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU6050_I2CSLV1_CTRL_REG	0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU6050_I2CSLV2_ADDR_REG	0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU6050_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU6050_I2CSLV2_CTRL_REG	0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU6050_I2CSLV3_ADDR_REG	0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU6050_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU6050_I2CSLV3_CTRL_REG	0X30	//IIC�ӻ�3���ƼĴ���
#define MPU6050_I2CSLV4_ADDR_REG	0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU6050_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU6050_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU6050_I2CSLV4_CTRL_REG	0X34	//IIC�ӻ�4���ƼĴ���
#define MPU6050_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU6050_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU6050_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU6050_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU6050_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU6050_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU6050_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU6050_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU6050_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU6050_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU6050_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU6050_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU6050_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU6050_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU6050_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU6050_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU6050_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU6050_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU6050_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU6050_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU6050_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU6050_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU6050_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU6050_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define MPU6050_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU6050_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define MPU6050_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU6050_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU6050_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2
#define MPU6050_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU6050_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU6050_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU6050_DEVICE_ID_REG		0X75	//����ID�Ĵ���

//���ٶ�����
#define  ACC_RANGE_2G  0  //2g	 
#define  ACC_RANGE_4G  1  //4g
#define  ACC_RANGE_8G  2  //8g	 
#define  ACC_RANGE_16G 3  //16g

//����������
#define  GYRO_RANGE_250  0  //250��/��	 
#define  GYRO_RANGE_500  1  //500��/��
#define  GYRO_RANGE_1000 2  //1000��/��	 
#define  GYRO_RANGE_2000 3  //2000��/��	 

//����
#define  DLPF_ACC184_GYRO188 1 //���ٶȴ���184Hz �����Ǵ���188Hz
#define  DLPF_ACC94_GYRO98   2 //���ٶȴ���94Hz �����Ǵ���98Hz
#define  DLPF_ACC44_GYRO42   3 //���ٶȴ���44Hz �����Ǵ���42Hz
#define  DLPF_ACC21_GYRO20   4 //���ٶȴ���21Hz �����Ǵ���20Hz
#define  DLPF_ACC10_GYRO10   5 //���ٶȴ���10Hz �����Ǵ���10Hz
#define  DLPF_ACC5_GYRO5     6 //���ٶȴ���5Hz �����Ǵ���5Hz 

//IIC������д������װ
u8 MPU6050_Write_Len(u8 dev_addr,u8 reg_addr,u8 len,u8 *data);
u8 MPU6050_Read_Len(u8 dev_addr,u8 reg_addr,u8 len,u8 *data);

//MPU6050��������ʼ��
void MPU6050_Init(void);

//���ò�������
void MPU6050_SetAccelRange(uint8_t range);  //MPU6050���ü��ٶ����� 
void MPU6050_SetGyroRange(uint8_t range);  //MPU6050��������������
void MPU6050_SetSmplRate(uint16_t smplrate);  //MPU6050���������ǲ�����
void MPU6050_SetDLPF(uint8_t lpf);  //MPU6050���õ�ͨ�˲�������

//��ȡ���ݺ���
float MPU6050_GetTempValue(void);	//MPU6050��ȡ�������¶�ֵ
int16_t MPU6050_GetAccelData_X(void);    //MPU6050��ȡX����ٶȼĴ������ֵ
int16_t MPU6050_GetAccelData_Y(void);    //MPU6050��ȡY����ٶȼĴ������ֵ
int16_t MPU6050_GetAccelData_Z(void);    //MPU6050��ȡZ����ٶȼĴ������ֵ
void MPU6050_GetAccelData(struct xyz_data *robot_accel_xyz_data);   //MPU6050��ȡ������ٶȼĴ������ֵ
int16_t MPU6050_GetGyroData_X(void);   //MPU6050��ȡX�������ǼĴ������ֵ
int16_t MPU6050_GetGyroData_Y(void);   //MPU6050��ȡY�������ǼĴ������ֵ
int16_t MPU6050_GetGyroData_Z(void);   //MPU6050��ȡZ�������ǼĴ������ֵ
void MPU6050_GetGyroData(struct xyz_data *robot_gyro_xyz_data);  //MPU6050��ȡ�����������ǼĴ������ֵ

//void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
//void mpu6050_send_data(float pitch,float roll,float yaw);




//////////////////////////////////////////////////////////////////
// DMP ����
//////////////////////////////////////////////////////////////////

#define DEFAULT_MPU_HZ  (100)  //���Ƶ��100Hz
#define q30   1073741824.0f //q30��ʽ,longתfloatʱ�ĳ���.

struct imu_data
{
	struct xyz_data gyro;
	struct xyz_data accel;
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
};

int MPU6050_DMP_Init(void);    //MPU6050 DMP��ʼ��
//void MPU6050_DMP_GetData(int16_t *data);  //MPU6050 MDP��ȡ�������ݣ���̬ŷ����
void MPU6050_DMP_GetData(struct imu_data *robot_imu_data);


#endif

