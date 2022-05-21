#include "main.h"
#include "bsp_pid.h"
#include "bsp_define.h"

#define LOG_TAG    "PID"
#include "bsp_log.h"

#define PID_SCALE  0.01f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限

int16_t motor_kp=300;  //电机转速PID-P
int16_t motor_ki=0;    //电机转速PID-I
int16_t motor_kd=900;  //电机转速PID-D

int16_t PID_Motor_Control(int8_t Motor_Num, int16_t speed_target, int16_t speed_current)
{
	static int16_t motor_pwm_out[4];
	static int32_t bias[4],bias_last[4],bias_integral[4] = {0};

	//获得偏差值
	bias[Motor_Num-1] = speed_target - speed_current;
	
	//计算偏差累加值
	bias_integral[Motor_Num-1] += bias[Motor_Num-1];
	
	//抗积分饱和
	if(bias_integral[Motor_Num-1] >  PID_INTEGRAL_UP) bias_integral[Motor_Num-1] =  PID_INTEGRAL_UP;
	if(bias_integral[Motor_Num-1] < -PID_INTEGRAL_UP) bias_integral[Motor_Num-1] = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out[Motor_Num-1] += motor_kp*bias[Motor_Num-1]*PID_SCALE + motor_kd*(bias[Motor_Num-1]-bias_last[Motor_Num-1])*PID_SCALE + motor_ki*bias_integral[Motor_Num-1]*PID_SCALE;
	
	//记录上次偏差
	bias_last[Motor_Num-1] = bias[Motor_Num-1];
	
	//限制最大输出
	if(motor_pwm_out[Motor_Num-1] > 2000)
		motor_pwm_out[Motor_Num-1] = 2000;
	if(motor_pwm_out[Motor_Num-1] < -2000)
		motor_pwm_out[Motor_Num-1] = -2000;
  
	//返回PWM控制值
	return motor_pwm_out[Motor_Num-1];
}	
