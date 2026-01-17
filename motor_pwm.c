#include "motor_pwm.h"
#include "main.h"   
#include "tim.h"   

//电机控制底层驱动

void Motor_Set(int16_t pwm_A, int16_t pwm_B)
{

    if(pwm_A >= 0)
    {
				//正转
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
				//反转
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        pwm_A = -pwm_A; 
    }

		//限幅保护
    if(pwm_A > 5500) pwm_A = 5500;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_A);



    if(pwm_B >= 0)
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
        pwm_B = -pwm_B;
    }

    if(pwm_B > 5500) pwm_B = 5500;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_B);
}


void Motor_Test_Loop(void)
{

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);


    for(int i = 0; i < 5500; i += 10)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, i); 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i); 
        HAL_Delay(1);
    }


    for(int j = 5500; j > 0; j -= 10)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, j);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, j);
        HAL_Delay(1);
    }
    

    Motor_Set(0, 0);
}
