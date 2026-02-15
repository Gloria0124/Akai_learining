//******************************************************************************
//  * @file           : vofa.c
//  ******************************************************************************

#include "vofa.h"
#include "usart.h"
#include "BLE.h"
#include <string.h>
#include "control.h" 
#include "stdlib.h"

extern float R,L;
extern PID_TypeDef PID_Turn;
extern float MPU6050_GYRO_OFFSET_X;
static uint8_t RxBuffer[32]; 
static uint8_t RxIndex = 0;  
extern uint8_t Rx_Char; 

void VOFA_JustFloat_Send(float *data, uint8_t channel_count)
{
    if (data == NULL || channel_count == 0) return;

    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    uint8_t tx_buf[64];

    uint8_t max_channels = (uint8_t)((sizeof(tx_buf) - sizeof(tail)) / sizeof(float));
    if (channel_count > max_channels) channel_count = max_channels;

    uint16_t payload_len = (uint16_t)(channel_count * sizeof(float));
    memcpy(tx_buf, (uint8_t*)data, payload_len);
    memcpy(tx_buf + payload_len, tail, sizeof(tail));

    BLE_SendArray(tx_buf, payload_len + sizeof(tail));
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
		else if (RxBuffer[0] == 'T') 
        {
            PID_Turn.Kp = atof((char*)&RxBuffer[1]);
        }
		else if (RxBuffer[0] == 'B')
		{
		MPU6050_GYRO_OFFSET_X = atof((char*)&RxBuffer[1]);
		}
		else if (RxBuffer[0] == 'M')
		{
		Target_Turn_Raw = atof((char*)&RxBuffer[1]);
		}
        RxIndex = 0;
    }
    else 
    {
        
        if (RxIndex < 30)
        {
            RxBuffer[RxIndex++] = ch;
        }
    }
}

