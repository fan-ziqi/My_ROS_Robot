#include "pid.h"
#include "delay.h"

#define PID_SCALE  0.01f  //PID����ϵ��
#define PID_INTEGRAL_UP 1000  //��������

int16_t motor_kp=600;  //���ת��PID-P
int16_t motor_ki=0;    //���ת��PID-I
int16_t motor_kd=400;  //���ת��PID-D

/**
  * @��  ��  ���A PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ ,��Χ����250��
  *          spd_current: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
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

/**
  * @��  ��  ���B PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
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

/**
  * @��  ��  ���C PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
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

/**
  * @��  ��  ���D PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
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


