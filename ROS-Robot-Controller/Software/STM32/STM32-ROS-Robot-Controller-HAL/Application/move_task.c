/*
	Copyright 2022 Fan Ziqi

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "main.h"
#include "bsp_define.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_pid.h"
#include "bsp_kinematics.h"

#define LOG_TAG    "MOVE"
#include "bsp_log.h"

int16_t encoder[4];	//�������ۼ�ֵ
int16_t encoder_delta[4];	//�������仯ֵ
int16_t encoder_delta_target[4] = {0};  //������Ŀ��仯ֵ
int16_t motor_pwm[4];  //���PWM

extern int16_t robot_odom[]; //��̼����ݣ�����ֵ�ͱ仯ֵ��x y yaw dx dy dyaw
extern int16_t robot_target_speed[];  //������Ŀ���ٶ� X Y Yaw

void move_task(void const * argument)
{
	while(1)
	{
		//��ȡ�������仯ֵ
		encoder_delta[0] = -(Encoder_Get_Counter(1) - ENCODER_MID_VALUE);
		encoder_delta[1] =  (Encoder_Get_Counter(2) - ENCODER_MID_VALUE);
		encoder_delta[2] = -(Encoder_Get_Counter(3) - ENCODER_MID_VALUE);
		encoder_delta[3] =  (Encoder_Get_Counter(4) - ENCODER_MID_VALUE);

		//���ñ������м�ֵ
		Encoder_Set_Counter(1, ENCODER_MID_VALUE);
		Encoder_Set_Counter(2, ENCODER_MID_VALUE);
		Encoder_Set_Counter(3, ENCODER_MID_VALUE);
		Encoder_Set_Counter(4, ENCODER_MID_VALUE);
			
		//����������ۼ�ֵ
		encoder[0] = encoder[0] + encoder_delta[0];
		encoder[1] = encoder[1] + encoder_delta[1];
		encoder[2] = encoder[2] + encoder_delta[2];
		encoder[3] = encoder[3] + encoder_delta[3];
		
		//�ٶ�����
		if(robot_target_speed[0] > ROBOT_LINEAR_SPEED_LIMIT)    robot_target_speed[0] = ROBOT_LINEAR_SPEED_LIMIT;
		if(robot_target_speed[0] < (-ROBOT_LINEAR_SPEED_LIMIT)) robot_target_speed[0] = (-ROBOT_LINEAR_SPEED_LIMIT);
		if(robot_target_speed[1] > ROBOT_LINEAR_SPEED_LIMIT)    robot_target_speed[1] = ROBOT_LINEAR_SPEED_LIMIT;
		if(robot_target_speed[1] < (-ROBOT_LINEAR_SPEED_LIMIT)) robot_target_speed[1] = (-ROBOT_LINEAR_SPEED_LIMIT);
		if(robot_target_speed[2] > ROBOT_ANGULAR_SPEED_LIMIT)    robot_target_speed[2] = ROBOT_ANGULAR_SPEED_LIMIT;
		if(robot_target_speed[2] < (-ROBOT_ANGULAR_SPEED_LIMIT)) robot_target_speed[2] = (-ROBOT_ANGULAR_SPEED_LIMIT);
			
		//�˶�ѧ����
		Kinematics_Forward(encoder,robot_odom);  //�����˶�ѧ����
		Kinematics_Inverse(robot_target_speed, encoder_delta_target);  //�����˶�ѧ����

		//���PID�ٶȿ���
		motor_pwm[0] = PID_Motor_Control(1, encoder_delta_target[0], encoder_delta[0]);   
		motor_pwm[1] = PID_Motor_Control(2, encoder_delta_target[1], encoder_delta[1]);   
		motor_pwm[2] = PID_Motor_Control(3, encoder_delta_target[2], encoder_delta[2]);   
		motor_pwm[3] = PID_Motor_Control(4, encoder_delta_target[3], encoder_delta[3]);  
										
		MOTOR_SetSpeed(1, motor_pwm[0]);
		MOTOR_SetSpeed(2, motor_pwm[1]);  
		MOTOR_SetSpeed(3, motor_pwm[2]);
		MOTOR_SetSpeed(4, motor_pwm[3]);
		
		osDelay(20); //����Ƶ��50HZ
	}
}
