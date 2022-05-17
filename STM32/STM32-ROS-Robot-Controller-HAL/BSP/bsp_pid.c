#include "main.h"
#include "bsp_pid.h"
#include "bsp_define.h"

#define LOG_TAG    "PID"
#include "bsp_log.h"

#define PID_SCALE  0.01f  //PID����ϵ��
#define PID_INTEGRAL_UP 1000  //��������

int16_t motor_kp=600;  //���ת��PID-P
int16_t motor_ki=0;    //���ת��PID-I
int16_t motor_kd=400;  //���ת��PID-D

int16_t PID_Motor_Control(int16_t speed_target, int16_t speed_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;

	//���ƫ��ֵ
	bias = speed_target - speed_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral += bias;
	
	//�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out += motor_kp*bias*PID_SCALE + motor_kd*(bias-bias_last)*PID_SCALE + motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
  
	//����PWM����ֵ
	return motor_pwm_out;
}	
