#include "main.h"
#include "bsp_kinematics.h"
#include "bsp_define.h"
#include <math.h>

#define LOG_TAG    "KINEMATICS"
#include "bsp_log.h"


//变量定义
float rx_plus_ry_cali = 0.3;
double pulse_per_meter = ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926);
double linear_correction_factor = 1.0;
double angular_correction_factor = 1.0;

int16_t robot_odom[6] = {0}; //里程计数据，绝对值和变化值，x y yaw dx dy dyaw
extern int16_t robot_target_speed[3];

/**
  * @简  述  机器人运动参数设置
  * @参  数  无
  * @返回值  无
  */
void Kinematics_Init(int16_t* robot_params)
{

	linear_correction_factor    = (float)robot_params[0]/1000;
	angular_correction_factor   = (float)robot_params[1]/1000;
	
	float r_x = D_X/2;
  float r_y = D_Y/2;
	rx_plus_ry_cali = (r_x + r_y)/angular_correction_factor;

	robot_odom[0]  = 0;
	robot_odom[1]  = 0;
	robot_odom[2]  = 0;

	//轮子转动一圈，移动的距离为轮子的周长WHEEL_DIAMETER*3.1415926
	//编码器产生的脉冲信号为ENCODER_RESOLUTION。
	//则电机编码器转一圈产生的脉冲信号除以轮子周长可得轮子前进1m的距离所对应编码器计数的变化
	pulse_per_meter = (float)(ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926))/linear_correction_factor;		
}

/**
  * @简  述  逆向运动学解析，底盘三轴速度->轮子速度
  * @参  数  input:  robot_target_speed[]  机器人三轴速度 m/s*1000
  *          output：encoder_delta_target[] 电机应达到的目标速度（一个PID控制周期内，电机编码器计数值的变化）
  * @返回值  无
  */
void Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float v_tx   = ((float)input[0])/1000;
	float v_ty   = ((float)input[1])/1000;
	float omega = ((float)input[2])/1000;
	static float v_w[4] = {0};
	
	v_w[0] = v_tx - v_ty - (rx_plus_ry_cali)*omega;
	v_w[1] = v_tx + v_ty + (rx_plus_ry_cali)*omega;
	v_w[2] = v_tx + v_ty - (rx_plus_ry_cali)*omega;
	v_w[3] = v_tx - v_ty + (rx_plus_ry_cali)*omega;

	//计算一个PID控制周期内，电机编码器计数值的变化
	output[0] = (int16_t)(v_w[0] * pulse_per_meter/PID_RATE);
	output[1] = (int16_t)(v_w[1] * pulse_per_meter/PID_RATE);
	output[2] = (int16_t)(v_w[2] * pulse_per_meter/PID_RATE);
	output[3] = (int16_t)(v_w[3] * pulse_per_meter/PID_RATE);
}


int32_t encoder_sum_current[4] = {0};
int32_t wheel_turns[4] = {0};
/**
  * @简  述  正向运动学解析，轮子编码值->底盘三轴里程计坐标
  * @参  数  input: encoder[]  编码器累加值
  *          output: robot_odom[] 三轴里程计 x y yaw
  * @返回值  无
  */
void Kinematics_Forward(int16_t* input, int16_t* output)
{
//	static double delta_v_integral[2];
	
	static double dv_w_times_dt[4]; //轮子瞬时变化量dxw=dvw*dt
	static double dv_t_times_dt[3]; //底盘瞬时变化量dxt=dvt*dt
	static int16_t encoder_sum[4];
	
	//将左面轮子编码器累加值乘以-1，以计算前进的距离
	encoder_sum[0] = -input[0];
	encoder_sum[1] = input[1];
	encoder_sum[2] = -input[2];
	encoder_sum[3] = input[3];
	
	//编码器计数溢出处理
	for(int i=0;i<4;i++)
	{
		if(encoder_sum[i] < ENCODER_LOW_WRAP && encoder_sum_current[i] > ENCODER_HIGH_WRAP)
		{
			wheel_turns[i]++;
		}
		else if(encoder_sum[i] > ENCODER_HIGH_WRAP && encoder_sum_current[i] < ENCODER_LOW_WRAP)
		{
			wheel_turns[i]--;
		}
		else
		{
			wheel_turns[i]=0;
		}
	}
	//printf("%d %d %d %d\r\n",wheel_turns[0],wheel_turns[1],wheel_turns[2],wheel_turns[3]);
	
	//将编码器数值转化为前进的距离，单位m
	for(int i=0;i<4;i++)
	{	
		dv_w_times_dt[i] = 1.0*(encoder_sum[i] + wheel_turns[i]*(ENCODER_MAX-ENCODER_MIN)-encoder_sum_current[i])/pulse_per_meter;
		encoder_sum_current[i] = encoder_sum[i];
	}
	
	//要计算坐标所以变回来
	dv_w_times_dt[0] = -dv_w_times_dt[0];
	dv_w_times_dt[1] =  dv_w_times_dt[1];
	dv_w_times_dt[2] = -dv_w_times_dt[2];
	dv_w_times_dt[3] =  dv_w_times_dt[3];
	
	//计算底盘坐标系(base_link)下x轴、y轴变化距离m与Yaw轴朝向变化rad 一段时间内的变化量
	dv_t_times_dt[0] = ( dv_w_times_dt[3] + dv_w_times_dt[2])/2.0;
	dv_t_times_dt[1] = ( dv_w_times_dt[2] - dv_w_times_dt[0])/2.0;
	dv_t_times_dt[2] = (-dv_w_times_dt[2] + dv_w_times_dt[1])/(2*rx_plus_ry_cali);
	
	//积分计算里程计坐标系(odom_frame)下的机器人X,Y,Yaw轴坐标
	//dx = ( vx*cos(theta) - vy*sin(theta) )*dt
	//dy = ( vx*sin(theta) + vy*cos(theta) )*dt
	output[0] += (int16_t)((cos((double)output[2]/1000)*dv_t_times_dt[0] - sin((double)output[2]/1000)*dv_t_times_dt[1])*1000);
	output[1] += (int16_t)((sin((double)output[2]/1000)*dv_t_times_dt[0] + cos((double)output[2]/1000)*dv_t_times_dt[1])*1000);
	output[2] += (int16_t)(dv_t_times_dt[2]*1000);
		
  //Yaw轴坐标变化范围控制-2pi -> 2pi
	if(output[2] > PI*1000)
	{
		output[2] -= 2*PI*1000;
	}
	else if(output[2] < -PI*1000)
	{
		output[2] += 2*PI*1000;
	}
		
	//发送机器人X轴y轴Yaw轴瞬时变化量，在ROS端除以时间
	output[3] = (int16_t)(dv_t_times_dt[0]*1000);
	output[4] = (int16_t)(dv_t_times_dt[1]*1000);
	output[5] = (int16_t)(dv_t_times_dt[2]*1000);
}

