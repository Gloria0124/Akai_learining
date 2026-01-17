#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

extern float Pitch;
extern float Balance_Out;
extern float Velocity_Out;
extern int PWM_Out;
extern float Gyro_Y;
extern float Mechanical_Angle;

void Get_Angle_Complementary(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void PID_Reset(void);
void Balance_Control_Process(void); // 平衡控制主处理函数

float Vertical_PID(float Angle, float Gyro);
float Velocity_PID(int Encoder_Left, int Encoder_Right);

#endif
