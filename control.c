//******************************************************************************
//  * @file           : control.c
//  ******************************************************************************

#include "control.h"
#include "main.h"      
#include "math.h"       
#include "Mpu6050.h"   
#include "motor_pwm.h"  
#include "encoder.h"    
#include "PID.h"
#include "usart.h"
#include "vofa.h"
extern float ReceiveData[10]; 
extern float MPU6050_GYRO_OFFSET_X;
float Roll;
float Mechanical_Angle = -1.1242;
float Encoder_Left, Encoder_Right;
float AveSpeed, DifSpeed;
float AvePWM, DifPWM;
float PWML, PWMR;
//遥控
float Target_Speed_Raw = 0; 
float Target_Turn_Raw = 0;  
float Filtered_Target_Speed = 0; 
float Filtered_Target_Turn = 0;  
float R,L;


//陀螺仪参数
MPU6050_DevTypeDef MPU6050_Data;
float K_Angle = 0.07f;     


//直立环
PID_TypeDef PID_Vertical = {
	.Kp = 162.067,
	.Kd =257.9502,
	.MaxOut = 4400,
	.MinOut = -4400,
	
};

//速度环
PID_TypeDef PID_Velocity = {
	.Kp = 3.1028,
	.Ki = 0.6509,
  .K = 10,
  .MaxOut = 500,
  .MinOut = -500,
  .MaxIntegral = 1400,
  .MinIntegral = -1400,
};

//转向环
PID_TypeDef PID_Turn = {
	.Kp = 0,
  .MaxOut = 600,
  .MinOut = -600,
  
};



float Velocity_Out = 0;

//遥控

void Remote_Control_Process(void)
{
    Filtered_Target_Speed = 0.9f * Filtered_Target_Speed + 0.1f * Target_Speed_Raw;
    Filtered_Target_Turn = 0.95f * Filtered_Target_Turn + 0.05f * Target_Turn_Raw;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t TIME1, TIME2;
    if (htim->Instance == TIM3) 
    {
			//读数
			MPU6050_ReadRawData(&MPU6050_Data);
			MPU6050_ComplementaryFilter(&MPU6050_Data, 10, DifSpeed);
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
				
				PID_Vertical.Actual_new = Roll - Mechanical_Angle;
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
				float Speed_Threshold = 40.0f; 
				static float Smooth_Velocity_Out = 0;
				
				PID_Velocity.Actual_new = AveSpeed;
				PID_Velocity.Target = Filtered_Target_Speed;
				
				
        if (fabs(AveSpeed) >= Speed_Threshold) 
        {
            PID_Velocity.Ki = 0.0f; 
            PID_Velocity.Error_add = 0.0f; 
        }
				PID_Cal(&PID_Velocity);
				
				Smooth_Velocity_Out = 0.7f * Smooth_Velocity_Out + 0.3f * PID_Velocity.Out;
				PID_Vertical.Target = Smooth_Velocity_Out;
				
				PID_Turn.Target = Filtered_Target_Turn;
				PID_Turn.Actual_new = DifSpeed;
				PID_Cal(&PID_Turn);
				DifPWM = PID_Turn.Out;
			}
			
			Remote_Control_Process();

		
      if(PWML > 0) PWML += 230;
      if(PWML < -0) PWML -= 230; 
			if(PWML > 4400) PWML = 4400;  
      if(PWML < -4400) PWML = -4400;

     	if(PWMR > 0) PWMR += 230;
      if(PWMR < -0) PWMR -= 230;
			if(PWMR > 4400) PWMR = 4400;  
      if(PWMR < -4400) PWMR = -4400;
		
			
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

//			VOFA_Send_Data(Roll, PID_Velocity.Target, PID_Velocity.Out, PID_Velocity.POut, PID_Velocity.IOut, PID_Velocity.DOut);
		}
}
