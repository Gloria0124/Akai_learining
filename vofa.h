#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h" 
#include "PID.h"

extern UART_HandleTypeDef huart3; 
extern PID_TypeDef PID_Vertical;
extern PID_TypeDef PID_Velocity;


void VOFA_JustFloat_Send(float *data, uint8_t channel_count);
void VOFA_Send_Data(float pitch, float speed, float pid_out_vel, float pid_out_vert, float gyro_y, float target_speed);
void VOFA_Get_Cmd(uint8_t ch);

#endif
