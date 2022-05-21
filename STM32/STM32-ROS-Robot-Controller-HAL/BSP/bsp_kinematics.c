#include "main.h"
#include "bsp_kinematics.h"
#include "bsp_define.h"
#include <math.h>

#define LOG_TAG    "KINEMATICS"
#include "bsp_log.h"


//��������
float rx_plus_ry_cali = 0.3;
double pulse_per_meter = ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926);
double linear_correction_factor = 1.0;
double angular_correction_factor = 1.0;

int16_t robot_odom[6] = {0}; //��̼����ݣ�����ֵ�ͱ仯ֵ��x y yaw dx dy dyaw
extern int16_t robot_target_speed[3];

/**
  * @��  ��  �������˶���������
  * @��  ��  ��
  * @����ֵ  ��
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

	//����ת��һȦ���ƶ��ľ���Ϊ���ӵ��ܳ�WHEEL_DIAMETER*3.1415926
	//�����������������ź�ΪENCODER_RESOLUTION��
	//����������תһȦ�����������źų��������ܳ��ɵ�����ǰ��1m�ľ�������Ӧ�����������ı仯
	pulse_per_meter = (float)(ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926))/linear_correction_factor;		
}

/**
  * @��  ��  �����˶�ѧ���������������ٶ�->�����ٶ�
  * @��  ��  input:  robot_target_speed[]  �����������ٶ� m/s*1000
  *          output��encoder_delta_target[] ���Ӧ�ﵽ��Ŀ���ٶȣ�һ��PID���������ڣ��������������ֵ�ı仯��
  * @����ֵ  ��
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

	//����һ��PID���������ڣ��������������ֵ�ı仯
	output[0] = (int16_t)(v_w[0] * pulse_per_meter/PID_RATE);
	output[1] = (int16_t)(v_w[1] * pulse_per_meter/PID_RATE);
	output[2] = (int16_t)(v_w[2] * pulse_per_meter/PID_RATE);
	output[3] = (int16_t)(v_w[3] * pulse_per_meter/PID_RATE);
}


int32_t encoder_sum_current[4] = {0};
int32_t wheel_turns[4] = {0};
/**
  * @��  ��  �����˶�ѧ���������ӱ���ֵ->����������̼�����
  * @��  ��  input: encoder[]  �������ۼ�ֵ
  *          output: robot_odom[] ������̼� x y yaw
  * @����ֵ  ��
  */
void Kinematics_Forward(int16_t* input, int16_t* output)
{
//	static double delta_v_integral[2];
	
	static double dv_w_times_dt[4]; //����˲ʱ�仯��dxw=dvw*dt
	static double dv_t_times_dt[3]; //����˲ʱ�仯��dxt=dvt*dt
	static int16_t encoder_sum[4];
	
	//���������ӱ������ۼ�ֵ����-1���Լ���ǰ���ľ���
	encoder_sum[0] = -input[0];
	encoder_sum[1] = input[1];
	encoder_sum[2] = -input[2];
	encoder_sum[3] = input[3];
	
	//�����������������
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
	
	//����������ֵת��Ϊǰ���ľ��룬��λm
	for(int i=0;i<4;i++)
	{	
		dv_w_times_dt[i] = 1.0*(encoder_sum[i] + wheel_turns[i]*(ENCODER_MAX-ENCODER_MIN)-encoder_sum_current[i])/pulse_per_meter;
		encoder_sum_current[i] = encoder_sum[i];
	}
	
	//Ҫ�����������Ա����
	dv_w_times_dt[0] = -dv_w_times_dt[0];
	dv_w_times_dt[1] =  dv_w_times_dt[1];
	dv_w_times_dt[2] = -dv_w_times_dt[2];
	dv_w_times_dt[3] =  dv_w_times_dt[3];
	
	//�����������ϵ(base_link)��x�ᡢy��仯����m��Yaw�ᳯ��仯rad һ��ʱ���ڵı仯��
	dv_t_times_dt[0] = ( dv_w_times_dt[3] + dv_w_times_dt[2])/2.0;
	dv_t_times_dt[1] = ( dv_w_times_dt[2] - dv_w_times_dt[0])/2.0;
	dv_t_times_dt[2] = (-dv_w_times_dt[2] + dv_w_times_dt[1])/(2*rx_plus_ry_cali);
	
	//���ּ�����̼�����ϵ(odom_frame)�µĻ�����X,Y,Yaw������
	//dx = ( vx*cos(theta) - vy*sin(theta) )*dt
	//dy = ( vx*sin(theta) + vy*cos(theta) )*dt
	output[0] += (int16_t)((cos((double)output[2]/1000)*dv_t_times_dt[0] - sin((double)output[2]/1000)*dv_t_times_dt[1])*1000);
	output[1] += (int16_t)((sin((double)output[2]/1000)*dv_t_times_dt[0] + cos((double)output[2]/1000)*dv_t_times_dt[1])*1000);
	output[2] += (int16_t)(dv_t_times_dt[2]*1000);
		
  //Yaw������仯��Χ����-2pi -> 2pi
	if(output[2] > PI*1000)
	{
		output[2] -= 2*PI*1000;
	}
	else if(output[2] < -PI*1000)
	{
		output[2] += 2*PI*1000;
	}
		
	//���ͻ�����X��y��Yaw��˲ʱ�仯������ROS�˳���ʱ��
	output[3] = (int16_t)(dv_t_times_dt[0]*1000);
	output[4] = (int16_t)(dv_t_times_dt[1]*1000);
	output[5] = (int16_t)(dv_t_times_dt[2]*1000);
}

