#include "main.h"
#include "bsp_pid.h"
#include "bsp_define.h"

#define LOG_TAG    "PID"
#include "bsp_log.h"

#define PID_SCALE  0.01f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限

int16_t motor_kp=600;  //电机转速PID-P
int16_t motor_ki=0;    //电机转速PID-I
int16_t motor_kd=400;  //电机转速PID-D

int16_t PID_Motor_Control(int16_t speed_target, int16_t speed_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;

	//获得偏差值
	bias = speed_target - speed_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
	//抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out += motor_kp*bias*PID_SCALE + motor_kd*(bias-bias_last)*PID_SCALE + motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
  
	//返回PWM控制值
	return motor_pwm_out;
}	
