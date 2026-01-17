//******************************************************************************
//  * @file           : vofa.c
//  ******************************************************************************

#include "vofa.h"
#include "usart.h"
#include "control.h" 
#include <stdlib.h> 
extern float R,L;
extern PID_TypeDef PID_Turn;

static uint8_t RxBuffer[32]; 
static uint8_t RxIndex = 0;  

void VOFA_JustFloat_Send(float *data, uint8_t channel_count)
{
    //VOFA+ 规定的结束暗号: 0x00, 0x00, 0x80, 0x7f
    static uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f}; 
    HAL_UART_Transmit(&huart3, (uint8_t*)data, channel_count * sizeof(float), 10);
    HAL_UART_Transmit(&huart3, tail, 4, 10);
}

void VOFA_Send_Data(float pitch, float speed, float pid_out_vel, float pid_out_vert, float gyro_y, float target_speed)
{
    float vofa_buffer[6];
    
    vofa_buffer[0] = pitch;         
    vofa_buffer[1] = speed;          
    vofa_buffer[2] = pid_out_vel;   
    vofa_buffer[3] = pid_out_vert;   
    vofa_buffer[4] = gyro_y;        
    vofa_buffer[5] = target_speed; 
   
    VOFA_JustFloat_Send(vofa_buffer, 6);
}




void VOFA_Get_Cmd(uint8_t ch)
{
    if (ch == '\n') 
    {
        RxBuffer[RxIndex] = 0; 
        
        //解析指令
        
        if (RxBuffer[0] == 'P') 
        {
            PID_Vertical.Kp = atof((char*)&RxBuffer[1]); 
        }
        else if (RxBuffer[0] == 'D') 
        {
            PID_Vertical.Kd = atof((char*)&RxBuffer[1]);
        }
        else if (RxBuffer[0] == 'S') 
        {
            PID_Velocity.Kp = atof((char*)&RxBuffer[1]);
        }
        else if (RxBuffer[0] == 'I') 
        {
            PID_Velocity.Ki = atof((char*)&RxBuffer[1]);
        }
				else if (RxBuffer[0] == 'R') 
        {
            R = atof((char*)&RxBuffer[1]);
        }
				else if (RxBuffer[0] == 'L') 
        {
            L = atof((char*)&RxBuffer[1]);
        }
				else if (RxBuffer[0] == 'T') 
        {
            PID_Turn.Kp = atof((char*)&RxBuffer[1]);
        }
        RxIndex = 0;
    }
    else 
    {
        
        if (RxIndex < 30) // 防止越界
        {
            RxBuffer[RxIndex++] = ch;
        }
    }
}

