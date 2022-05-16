#ifndef BSP_IMU_H
#define BSP_IMU_H
#include "bsp_define.h"

#include <stdio.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "bsp_iic.h"

//////////////////////////////////////////////////////////////////
// MPU6050 部分
//////////////////////////////////////////////////////////////////

struct xyz_data
{
	int16_t x;
	int16_t y;
	int16_t z;
};

//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define	MPU6050_ADDR    0x68    //MPU6050地址

//MPU6050寄存器地址
//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
#define MPU6050_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU6050_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU6050_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU6050_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU6050_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU6050_CFG_REG				0X1A	//配置寄存器
#define MPU6050_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU6050_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU6050_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU6050_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU6050_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU6050_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU6050_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU6050_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU6050_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU6050_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU6050_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU6050_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU6050_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU6050_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU6050_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU6050_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU6050_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU6050_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU6050_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU6050_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU6050_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU6050_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU6050_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU6050_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU6050_INT_EN_REG			0X38	//中断使能寄存器
#define MPU6050_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU6050_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU6050_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU6050_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU6050_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU6050_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU6050_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU6050_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU6050_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU6050_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU6050_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU6050_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU6050_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU6050_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU6050_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU6050_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU6050_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU6050_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU6050_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU6050_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU6050_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU6050_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU6050_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU6050_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU6050_PWR_MGMT2_REG		0X6C	//电源管理寄存器2
#define MPU6050_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU6050_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU6050_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU6050_DEVICE_ID_REG		0X75	//器件ID寄存器

//加速度量程
#define  ACC_RANGE_2G  0  //2g	 
#define  ACC_RANGE_4G  1  //4g
#define  ACC_RANGE_8G  2  //8g	 
#define  ACC_RANGE_16G 3  //16g

//陀螺仪量程
#define  GYRO_RANGE_250  0  //250度/秒	 
#define  GYRO_RANGE_500  1  //500度/秒
#define  GYRO_RANGE_1000 2  //1000度/秒	 
#define  GYRO_RANGE_2000 3  //2000度/秒	 

//带宽
#define  DLPF_ACC184_GYRO188 1 //加速度带宽184Hz 陀螺仪带宽188Hz
#define  DLPF_ACC94_GYRO98   2 //加速度带宽94Hz 陀螺仪带宽98Hz
#define  DLPF_ACC44_GYRO42   3 //加速度带宽44Hz 陀螺仪带宽42Hz
#define  DLPF_ACC21_GYRO20   4 //加速度带宽21Hz 陀螺仪带宽20Hz
#define  DLPF_ACC10_GYRO10   5 //加速度带宽10Hz 陀螺仪带宽10Hz
#define  DLPF_ACC5_GYRO5     6 //加速度带宽5Hz 陀螺仪带宽5Hz 

//IIC连续读写函数封装
u8 MPU6050_Write_Len(u8 dev_addr,u8 reg_addr,u8 len,u8 *data);
u8 MPU6050_Read_Len(u8 dev_addr,u8 reg_addr,u8 len,u8 *data);

//MPU6050传感器初始化
void MPU6050_Init(void);

//设置参数函数
void MPU6050_SetAccelRange(uint8_t range);  //MPU6050设置加速度量程 
void MPU6050_SetGyroRange(uint8_t range);  //MPU6050设置陀螺仪量程
void MPU6050_SetSmplRate(uint16_t smplrate);  //MPU6050设置陀螺仪采样率
void MPU6050_SetDLPF(uint8_t lpf);  //MPU6050设置低通滤波器带宽

//获取数据函数
float MPU6050_GetTempValue(void);	//MPU6050获取传感器温度值
int16_t MPU6050_GetAccelData_X(void);    //MPU6050获取X轴加速度寄存器输出值
int16_t MPU6050_GetAccelData_Y(void);    //MPU6050获取Y轴加速度寄存器输出值
int16_t MPU6050_GetAccelData_Z(void);    //MPU6050获取Z轴加速度寄存器输出值
void MPU6050_GetAccelData(struct xyz_data *robot_accel_xyz_data);   //MPU6050获取三轴加速度寄存器输出值
int16_t MPU6050_GetGyroData_X(void);   //MPU6050获取X轴陀螺仪寄存器输出值
int16_t MPU6050_GetGyroData_Y(void);   //MPU6050获取Y轴陀螺仪寄存器输出值
int16_t MPU6050_GetGyroData_Z(void);   //MPU6050获取Z轴陀螺仪寄存器输出值
void MPU6050_GetGyroData(struct xyz_data *robot_gyro_xyz_data);  //MPU6050获取三轴轴陀螺仪寄存器输出值

//void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
//void mpu6050_send_data(float pitch,float roll,float yaw);




//////////////////////////////////////////////////////////////////
// DMP 部分
//////////////////////////////////////////////////////////////////

#define DEFAULT_MPU_HZ  (100)  //输出频率100Hz
#define q30   1073741824.0f //q30格式,long转float时的除数.

struct imu_data
{
	struct xyz_data gyro;
	struct xyz_data accel;
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
};

int MPU6050_DMP_Init(void);    //MPU6050 DMP初始化
//void MPU6050_DMP_GetData(int16_t *data);  //MPU6050 MDP获取解算数据，姿态欧拉角
void MPU6050_DMP_GetData(struct imu_data *robot_imu_data);


#endif

