//******************************************************************************
//  * @file           : encoder.c
//  ******************************************************************************

#include "tim.h"
#include "main.h"
#include "motor_pwm.h"
#include "Mpu6050.h"
void Encoder_Start(void)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);   
	__HAL_TIM_SET_COUNTER(&htim4,0);
	//开启编码器中断
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
}


int Read_Speed(int TIMx)
{
	int Encoder_Value = 0;
	switch(TIMx)
	{
		case 2: 
			
			Encoder_Value =-(short) __HAL_TIM_GET_COUNTER(&htim2);     
			__HAL_TIM_SET_COUNTER(&htim2,0);                  
			break;

		case 4: 
			Encoder_Value =(short) __HAL_TIM_GET_COUNTER(&htim4);
			__HAL_TIM_SET_COUNTER(&htim4,0);
			break;
		
		default:
			Encoder_Value = 0;
	}	
	return Encoder_Value;
}
