#include "main.h"
#include "bsp_imu.h"
#include "bsp_delay.h"

#define LOG_TAG    "IMU"
#include "bsp_log.h"

//////////////////////////////////////////////////////////////////
// MPU6050 部分
//////////////////////////////////////////////////////////////////

//IIC写一个字节
static void MPU6050_Write_Byte(uint8_t reg_address, uint8_t data)
{
	MPU6050_Write_Len(MPU6050_ADDR,reg_address,1,&data);
}

//IIC读一个字节
static void MPU6050_Read_Byte(uint8_t reg_address, uint8_t *pdata)
{
	MPU6050_Read_Len(MPU6050_ADDR,reg_address,1,pdata);
}


/**
  * @简  述  MPU6050传感器初始化
  * @参  数  无	  
  * @返回值  无
  */
void MPU6050_Init(void)
{	
	IIC_Init();//初始化IIC总线
	
	//配置MPU6050寄存器  
	MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG,0X80);	//复位MPU6050
	osDelay(100);
	MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG,0x00); //唤醒MPU6050 
	
	MPU6050_SetGyroRange(GYRO_RANGE_2000); //陀螺仪量程 ±2000dps
	MPU6050_SetAccelRange(ACC_RANGE_2G); //加速计量程 ±2g
	
	MPU6050_Write_Byte(MPU6050_INT_EN_REG,0X00);	//关闭所有中断
	MPU6050_Write_Byte(MPU6050_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU6050_Write_Byte(MPU6050_FIFO_EN_REG,0xFF); //关闭FIFO
	MPU6050_Write_Byte(MPU6050_INTBP_CFG_REG,0x80); //INT引脚低电平有效
	
	//检验器件ID
	u8 ID;
	MPU6050_Read_Byte(MPU6050_DEVICE_ID_REG, &ID);
	if(ID==MPU6050_ADDR) //器件ID正确
	{
		MPU6050_Write_Byte(MPU6050_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU6050_Write_Byte(MPU6050_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU6050_SetSmplRate(50); //设置采样率为50Hz
		
		osDelay(100);  //等待传感器稳定
		
		if(MPU6050_DMP_Init())	//DMP初始化
		{
			LOG_I("MPU6050 Init Success, ID=0x%x\r\n",ID);
		}
		else
		{
			LOG_E("MPU6050 Init Failed: DMP Failed");
		}
 	}
	else
	{
		LOG_E("MPU6050 Init Failed, ID=0x%x\r\n",ID);
	}
}


/**
  * @简  述  设置MPU6050加速度传感器量程 
  * @参  数  range： 加速度量程 0,±2g;1,±4g;2,±8g;3,±16g
  *          可设置的加速度量程ACC_RANGE_2G、ACC_RANGE_4G、ACC_RANGE_8G、ACC_RANGE_16G
  * @返回值  无
  */
void MPU6050_SetAccelRange(uint8_t range)
{
	MPU6050_Write_Byte(MPU6050_ACCEL_CFG_REG,range<<3);
}


/**
  * @简  述  设置MPU6050陀螺仪传感器满量程范围
  * @参  数  range 陀螺仪量程 0,±250dps;1,±500dps;2,±1000dps;3,±2000dps (degree per second)
  *          可设置的陀螺仪量程GYRO_RANGE_250、GYRO_RANGE_500、GYRO_RANGE_1000、GYRO_RANGE_2000	
  * @返回值  无
  */
void MPU6050_SetGyroRange(uint8_t range)
{
	MPU6050_Write_Byte(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @简  述  MPU6050设置低通滤波器带宽
  * @参  数  lpf:数字低通滤波频率(Hz)
  *          可设置的带宽： DLPF_ACC184_GYRO188、DLPF_ACC94_GYRO98、DLPF_ACC44_GYRO42、
  *                        DLPF_ACC21_GYRO20、DLPF_ACC10_GYRO10、DLPF_ACC5_GYRO5
  * @返回值  无
  */
void MPU6050_SetDLPF(uint8_t lpf)
{
	u8 data=0;
	if(lpf>=188)
		data=DLPF_ACC184_GYRO188;
	else if(lpf>=98)
		data=DLPF_ACC94_GYRO98;
	else if(lpf>=42)
		data=DLPF_ACC44_GYRO42;
	else if(lpf>=20)
		data=DLPF_ACC21_GYRO20;
	else if(lpf>=10)
		data=DLPF_ACC10_GYRO10;
	else 
		data=DLPF_ACC5_GYRO5; 
	MPU6050_Write_Byte(MPU6050_CFG_REG,data);//设置数字低通滤波器  
}

/**
  * @简  述  MPU6050设置陀螺仪采样率
  * @参  数  smplrate 陀螺仪采样率，范围10~1000Hz	  
  * @返回值  无
  */
void MPU6050_SetSmplRate(uint16_t smplrate)
{	
	if(smplrate>1000)
		smplrate = 1000;
	if(smplrate<10)
		smplrate = 10;
	
	MPU6050_Write_Byte(MPU6050_SAMPLE_RATE_REG,(uint8_t)(1000/smplrate -1));	//设置数字低通滤波器
	MPU6050_SetDLPF(smplrate/2);	//自动设置LPF为采样率的一半
}

/**
  * @简  述  MPU6050获取传感器温度值
  * @参  数  无	  
  * @返回值  传感器温度值。
  */
float MPU6050_GetTempValue(void)
{	
	uint8_t buf[2];
	int16_t tmp;

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_TEMP_OUTH_REG,2,buf);

	tmp = (buf[0]<<8)| buf[1];
	
	return ( 36.53f + ((double)tmp/340.0f) );	
}

/**
  * @简  述  MPU6050获取X轴加速度寄存器输出值
  * @参  数  无	  
  * @返回值  X轴加速度寄存器数据。
  */
int16_t MPU6050_GetAccelData_X(void)
{
	uint8_t buf[2];
	
	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_ACCEL_XOUTH_REG,2,buf);

	return ((buf[0]<<8) | buf[1]);	
}

/**
  * @简  述  MPU6050获取Y轴加速度寄存器输出值
  * @参  数  无	  
  * @返回值  Y轴加速度寄存器数据。
  */
int16_t MPU6050_GetAccelData_Y(void)
{	
	uint8_t buf[2];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_ACCEL_YOUTH_REG,2,buf);

	return ((buf[0]<<8) | buf[1]);	
}

/**
  * @简  述  MPU6050获取Z轴加速度寄存器输出值
  * @参  数  无	  
  * @返回值  Z轴加速度寄存器数据。
  */
int16_t MPU6050_GetAccelData_Z(void)
{	
	uint8_t buf[2];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_ACCEL_ZOUTH_REG,2,buf);

	return ((buf[0]<<8) | buf[1]);		
}

/**
  * @简  述  MPU6050获取三轴加速度寄存器输出值(带符号的原始值)
  * @参  数  pbuf：读取的数据缓冲区指针 
  * @返回值  无
  */
void MPU6050_GetAccelData(struct xyz_data *robot_accel_xyz_data)
{	
	uint8_t buf[6];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_ACCEL_XOUTH_REG,6,buf);
	
    robot_accel_xyz_data->x = (buf[0] << 8) | buf[1];
    robot_accel_xyz_data->y = (buf[2] << 8) | buf[3];
    robot_accel_xyz_data->z = (buf[4] << 8) | buf[5];	
}


/**
  * @简  述  MPU6050获取X轴陀螺仪寄存器输出值
  * @参  数  无	  
  * @返回值  X轴陀螺仪寄存器数据。
  */
int16_t MPU6050_GetGyroData_X(void)
{

	uint8_t buf[2];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_GYRO_XOUTH_REG,2,buf);

	return ((buf[0]<<8) | buf[1]);		
}

/**
  * @简  述  MPU6050获取Y轴陀螺仪寄存器输出值
  * @参  数  无	  
  * @返回值  Y轴陀螺仪寄存器数据。
  */
int16_t MPU6050_GetGyroData_Y(void)
{	
	uint8_t buf[2];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_GYRO_YOUTH_REG,2,buf);

	return ((buf[0]<<8) | buf[1]);
}

/**
  * @简  述  MPU6050获取Z轴陀螺仪寄存器输出值
  * @参  数  无	  
  * @返回值  Z轴陀螺仪寄存器数据。
  */
int16_t MPU6050_GetGyroData_Z(void)
{	
	uint8_t buf[2];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_GYRO_ZOUTH_REG,2,buf);

	return ((buf[0]<<8) | buf[1]);
}

/**
  * @简  述  MPU6050获取三轴轴陀螺仪寄存器输出值(带符号的原始值)
  * @参  数  pbuf：读取的数据缓冲区指针 	  
  * @返回值  无
  */
void MPU6050_GetGyroData(struct xyz_data *robot_gyro_xyz_data)
{	
	uint8_t buf[6];

	MPU6050_Read_Len(MPU6050_ADDR,MPU6050_GYRO_XOUTH_REG,6,buf);
	
	robot_gyro_xyz_data->x = (buf[0] << 8) | buf[1];
	robot_gyro_xyz_data->y = (buf[2] << 8) | buf[3];
	robot_gyro_xyz_data->z = (buf[4] << 8) | buf[5];	
}										


//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU6050_Write_Len(u8 dev_addr,u8 reg_addr,u8 len,u8 *data)
{
	u8 i; 
  IIC_Start(); 
	IIC_Send_Byte((dev_addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(reg_addr);	//写寄存器地址
	IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(data[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
  IIC_Stop();	 
	return 0;	
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU6050_Read_Len(u8 dev_addr,u8 reg_addr,u8 len,u8 *data)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((dev_addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(reg_addr);	//写寄存器地址
	IIC_Wait_Ack();		//等待应答
	IIC_Start();
	IIC_Send_Byte((dev_addr<<1)|1);//发送器件地址+读命令	
  IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)
			*data=IIC_Read_Byte(0);//读数据,发送nACK 
		else 
			*data=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		data++; 
	}    
  IIC_Stop();	//产生一个停止条件 
	return 0;	
}

//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
  IIC_Start(); 
	IIC_Send_Byte((MPU6050_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
	IIC_Send_Byte(reg);	//写寄存器地址
	IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
  IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
  IIC_Start(); 
	IIC_Send_Byte((MPU6050_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(reg);	//写寄存器地址
	IIC_Wait_Ack();		//等待应答
	IIC_Start();
	IIC_Send_Byte((MPU6050_ADDR<<1)|1);//发送器件地址+读命令	
	IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
  IIC_Stop();			//产生一个停止条件 
	return res;		
}

////fun:功能字. 0XA0~0XAF
////data:数据缓存区,最多28字节!!
////len:data区有效数据个数
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//    u8 send_buf[32];
//    u8 i;
//    if(len>28)return;    //最多28字节数据
//    send_buf[len+3]=0;  //校验数置零
//    send_buf[0]=0X88;   //帧头
//    send_buf[1]=fun;    //功能字
//    send_buf[2]=len;    //数据长度
//    for(i=0;i<len;i++)send_buf[3+i]=data[i];         //复制数据
//    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];    //计算校验和
//    for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);   //发送数据到串口1
//}
// 
//void mpu6050_send_data(float pitch,float roll,float yaw)
//{
//    u8 tbuf[16];
//    unsigned char *p;
//    p=(unsigned char *)&pitch;
//    tbuf[0]=(unsigned char)(*(p+3));
//    tbuf[1]=(unsigned char)(*(p+2));
//    tbuf[2]=(unsigned char)(*(p+1));
//    tbuf[3]=(unsigned char)(*(p+0));
//     
//    p=(unsigned char *)&roll;
//    tbuf[4]=(unsigned char)(*(p+3));
//    tbuf[5]=(unsigned char)(*(p+2));
//    tbuf[6]=(unsigned char)(*(p+1));
//    tbuf[7]=(unsigned char)(*(p+0));
//     
//    p=(unsigned char *)&yaw;
//    tbuf[8]=(unsigned char)(*(p+3));
//    tbuf[9]=(unsigned char)(*(p+2));
//    tbuf[10]=(unsigned char)(*(p+1));
//    tbuf[11]=(unsigned char)(*(p+0));
//     
//    usart1_niming_report(0XA2,tbuf,12);//自定义帧,0XA2
//}  




//////////////////////////////////////////////////////////////////
// DMP 部分
//////////////////////////////////////////////////////////////////

short sensors;
float pitch,roll,yaw; 


//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

//MPU6050自测试
//返回值:0,正常
//    其他,失败
static u8 run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
	
    if (result == 0x3) 
		{
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
		
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
		
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
				return 0;
    }
		else return 1;
}

//方向转换
static uint16_t inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//陀螺仪方向控制
static  uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
		/*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
	
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}				


/**
  * @简  述  MPU6050 DMP初始化
  * @参  数  无	  
  * @返回值  无
  */
int MPU6050_DMP_Init(void)
{
	uint8_t res=0;
	
	int ok = 1;

	if(mpu_init() == 0)
	{
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
		if(res&&ok)
		{
			ok = 0;
			LOG_E("mpu_set_sensor error\r\n");
			return 0;
		}
		
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
		if(res&&ok)
		{
			ok = 0;
			LOG_E("mpu_configure_fifo error\r\n"); 
			return 0;
		}
		
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	//设置采样率
		if(res&&ok)
		{
			ok = 0;
			LOG_E("mpu_set_sample_rate error\r\n");
			return 0;
		}
		
		res=dmp_load_motion_driver_firmware();		//加载dmp固件
		if(res&&ok)
		{
			ok = 0;
			LOG_E("dmp_load_motion_driver_firmware error\r\n"); 
			return 0;
		}
		
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res&&ok)
		{
			ok = 0;
			LOG_E("dmp_set_orientation error\r\n");
			return 0;
		}
		
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res&&ok)
		{
			ok = 0;
			LOG_E("dmp_enable_feature error\r\n");
			return 0;
		}
		
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res&&ok)
		{
			ok = 0;
			LOG_E("dmp_set_fifo_rate error\r\n");
			return 0;
		}
		
		res=run_self_test();  //自检
		if(res&&ok)
		{
			ok = 0;
			LOG_E("run_self_test error\r\n");
			return 0;
		}
		
		res=mpu_set_dmp_state(1);  //使能DMP
		if(res&&ok)
		{
			ok = 0;
			LOG_E("mpu_set_dmp_state error\r\n"); 
			return 0;
		}
		
		if(ok)
		{
			LOG_D("MPU6050_DMP Init Success\r\n");
			return 1;
		}
	}
	return 0;
}


void MPU6050_DMP_GetData(struct imu_data *robot_imu_data)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 
	
	robot_imu_data->gyro.x = gyro[0];
	robot_imu_data->gyro.y = gyro[1];
	robot_imu_data->gyro.z = gyro[2];
	
	robot_imu_data->accel.x = accel[0];
	robot_imu_data->accel.y = accel[1];
	robot_imu_data->accel.z = accel[2];
	
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	
	if(sensors & INV_WXYZ_QUAT) 
	{
		//q30格式转换为浮点数
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		
		//计算得到俯仰角/横滚角/航向角
		pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
		
		robot_imu_data->pitch = pitch * 100;		
		robot_imu_data->roll  = roll  * 100;	
	  robot_imu_data->yaw   = yaw   * 100;	
	}
	
}



/**
  * @简  述  MPU6050 DMP获取解算数据，姿态欧拉角
  * @参  数  无	  
  * @返回值  无
  * @全  局
	data[0-2] 陀螺仪
  data[3-5] 加速度
  data[6-8] 欧拉角 横滚，俯仰，航向，实际角度扩大100倍
  pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
	roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
	yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
  */
//void MPU6050_DMP_GetData(int16_t *data)
//{
//	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
//	unsigned long sensor_timestamp;
//	unsigned char more;
//	long quat[4];
//	dmp_read_fifo(&data[0], &data[3], quat, &sensor_timestamp, &sensors, &more);
//	
//	if ( sensors & INV_WXYZ_QUAT )
//	{
//		 q0 = quat[0] / q30; //q30格式转换为浮点数
//		 q1 = quat[1] / q30;
//		 q2 = quat[2] / q30;
//		 q3 = quat[3] / q30;		
//		 
//		 pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//pitch
//		 roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; //roll
//		 yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
//		
//		 data[6] = roll*100;		
//		 data[7] = pitch*100;	
//		 data[8] = yaw*100;	
//	}
//}


