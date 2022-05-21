#include "main.h"
#include "bsp_pid.h"
#include "bsp_define.h"

#define LOG_TAG    "PID"
#include "bsp_log.h"

#define PID_SCALE  0.01f  //PID����ϵ��
#define PID_INTEGRAL_UP 1000  //��������

int16_t motor_kp=300;  //���ת��PID-P
int16_t motor_ki=0;    //���ת��PID-I
int16_t motor_kd=900;  //���ת��PID-D

int16_t PID_Motor_Control(int8_t Motor_Num, int16_t speed_target, int16_t speed_current)
{
	static int16_t motor_pwm_out[4];
	static int32_t bias[4],bias_last[4],bias_integral[4] = {0};

	//���ƫ��ֵ
	bias[Motor_Num-1] = speed_target - speed_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral[Motor_Num-1] += bias[Motor_Num-1];
	
	//�����ֱ���
	if(bias_integral[Motor_Num-1] >  PID_INTEGRAL_UP) bias_integral[Motor_Num-1] =  PID_INTEGRAL_UP;
	if(bias_integral[Motor_Num-1] < -PID_INTEGRAL_UP) bias_integral[Motor_Num-1] = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out[Motor_Num-1] += motor_kp*bias[Motor_Num-1]*PID_SCALE + motor_kd*(bias[Motor_Num-1]-bias_last[Motor_Num-1])*PID_SCALE + motor_ki*bias_integral[Motor_Num-1]*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last[Motor_Num-1] = bias[Motor_Num-1];
	
	//����������
	if(motor_pwm_out[Motor_Num-1] > 2000)
		motor_pwm_out[Motor_Num-1] = 2000;
	if(motor_pwm_out[Motor_Num-1] < -2000)
		motor_pwm_out[Motor_Num-1] = -2000;
  
	//����PWM����ֵ
	return motor_pwm_out[Motor_Num-1];
}	
