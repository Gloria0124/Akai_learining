//******************************************************************************
//  * @file           : control.c
//  ******************************************************************************

//巡线按钮
#define LINE_BTN_GPIO_PORT GPIOA
#define LINE_BTN_PIN GPIO_PIN_9
#define LINE_BTN_ACTIVE_STATE GPIO_PIN_RESET
//允许修正速度阈值
#define LINE_SPEED_ENABLE_TH 2.0f
//偏差死区
#define LINE_OFFSET_DEADZONE 3
//偏差放大系数
#define LINE_TURN_SCALE 0.7f

#define LINE_TURN_MAX 150.0f
#define LINE_SPEED_TARGET 1350.0f

#include "control.h"
#include "main.h"      
#include "math.h"       
#include "Mpu6050.h"   
#include "motor_pwm.h"  
#include "encoder.h"    
#include "PID.h"
#include "usart.h"
#include "vofa.h"
#include "line_sensor.h"

extern float ReceiveData[10]; 
float MPU6050_GYRO_OFFSET_X = -525.25;
float Roll;
float Encoder_Left, Encoder_Right;
float AveSpeed, DifSpeed;
float AvePWM, DifPWM;
float PWML, PWMR;
float Velocity_Out = 0;
float Speed_Threshold = 1800.0f; 
static float Smooth_Velocity_Out = 0;

//遥控
float Target_Speed_Raw = 0; 
float Target_Turn_Raw = 0;  
float Filtered_Target_Speed = 0; 
float Filtered_Target_Turn = 0;  
float R,L;


//陀螺仪参数
MPU6050_DevTypeDef MPU6050_Data; 

//直立环
PID_TypeDef PID_Vertical = {
	.Kp = 152.8349,
	.Kd =253.0162,
	.MaxOut = 4400,
	.MinOut = -4400,
	
};

//速度环
PID_TypeDef PID_Velocity = {
	.Kp =3.4128,
	.Ki = 0.0486,
  	.K = 10,
  	.MaxOut = 18,
  	.MinOut = -18,
 	.MaxIntegral = 400,
  	.MinIntegral = -400,
}; 

//转向环
PID_TypeDef PID_Turn = {
	.Kp = 4.0,
  	.MaxOut = 600,
  	.MinOut = -600,
  
};


//遥控

void Remote_Control_Process(void)
{

    Filtered_Target_Speed = 0.85f * Filtered_Target_Speed + 0.15f * Target_Speed_Raw;
    Filtered_Target_Turn = 0.92f * Filtered_Target_Turn + 0.08f * Target_Turn_Raw;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t TIME1, TIME2;
	static uint8_t TIME_LINE;
	//巡线状态判断
	static uint8_t line_mode_enabled;
	static uint8_t line_btn_last;
    if (htim->Instance == TIM3) 
    {
			//读数
			MPU6050_ReadRawData(&MPU6050_Data);
			MPU6050_KalmanFilter(&MPU6050_Data, 10, DifSpeed);
			Roll = MPU6050_Data.Roll;
			
			//PID直立环
			TIME1++;
			if(TIME1 >= 1)
			{
				TIME1 = 0;
				
				Encoder_Left =(float)Read_Speed(2);
				Encoder_Right =-(float)Read_Speed(4);
				
				AveSpeed = (Encoder_Left + Encoder_Right) / 2.0;
				AveSpeed = (fabs(AveSpeed) <= 1.0f) ? 0.0f : AveSpeed; 
				
				DifSpeed = Encoder_Left - Encoder_Right;
				
				PID_Vertical.Actual_new = Roll;
				PID_Cal(&PID_Vertical);
				
				AvePWM = PID_Vertical.Out;
				DifPWM = PID_Turn.Out;
				PWML = AvePWM - DifPWM;
				PWMR = AvePWM + DifPWM;
			}
			
			TIME2++;
			if(TIME2 >= 5)
			{
				TIME2 = 0;
				float Vel_Input_DeadZone = 2.0f; 
				if (fabs(AveSpeed) < Vel_Input_DeadZone)
				{
					PID_Velocity.Actual_new = 0.0f; 
				}
				else
				{
					PID_Velocity.Actual_new = AveSpeed;
				}
				
				if (line_mode_enabled)
				{
					PID_Velocity.Target = LINE_SPEED_TARGET;
				}
				else
				{
					PID_Velocity.Target = Filtered_Target_Speed;
				}
				
				

        		if (fabs(AveSpeed) >= Speed_Threshold) 
        		{
            		PID_Velocity.Ki = 0.0f; 
            		PID_Velocity.Error_add = 0.0f; 
        		}
					PID_Cal(&PID_Velocity);
					Smooth_Velocity_Out = 0.75f * Smooth_Velocity_Out + 0.25f * PID_Velocity.Out;
					PID_Vertical.Target = Smooth_Velocity_Out;
				
					float line_turn = 0.0f;
					if (line_mode_enabled &&
						(fabs(Target_Speed_Raw) > LINE_SPEED_ENABLE_TH || fabs(AveSpeed) > LINE_SPEED_ENABLE_TH))
					{
						int8_t offset = LineSensor_GetOffset();
						if (offset > -LINE_OFFSET_DEADZONE && offset < LINE_OFFSET_DEADZONE)
						{
							offset = 0;
						}
						line_turn = (float)offset * LINE_TURN_SCALE;
						if (line_turn > LINE_TURN_MAX)
						{
							line_turn = LINE_TURN_MAX;
						}
						else if (line_turn < -LINE_TURN_MAX)
						{
							line_turn = -LINE_TURN_MAX;
						}
					}

					PID_Turn.Target = line_mode_enabled ? line_turn : Filtered_Target_Turn;
					PID_Turn.Actual_new = DifSpeed;
					PID_Cal(&PID_Turn);
					DifPWM = PID_Turn.Out;
			}

				TIME_LINE++;
				if (TIME_LINE >= 2)
				{
					TIME_LINE = 0;
					LineSensor_Update();
					uint8_t btn_now = (HAL_GPIO_ReadPin(LINE_BTN_GPIO_PORT, LINE_BTN_PIN) == LINE_BTN_ACTIVE_STATE);
					if (btn_now && !line_btn_last)
					{
						line_mode_enabled = !line_mode_enabled;
					}
					line_btn_last = btn_now;
				}
			Remote_Control_Process();

		
      		if(PWML > 0) PWML += 150;
      		if(PWML < 0) PWML -= 150; 
     		if(PWMR > 0) PWMR += 180;
      		if(PWMR < 0) PWMR -= 180;
			
      		if(Roll < -35 || Roll > 35) 
      		{				
        		Motor_Set(0, 0);
				PID_Vertical.Error_add = 0;
				PID_Velocity.Error_add = 0;
				PID_Turn.Error_add = 0; 
				Filtered_Target_Speed = 0;
      		}
      		else
      		{
				Motor_Set(PWML,PWMR);
      		}

			VOFA_Send_Data(Roll, PID_Velocity.Target, PID_Velocity.Out, PWMR, PWML, PID_Velocity.DOut);
	}
}
