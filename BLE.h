#ifndef __BLE_H
#define __BLE_H

#include "usart.h" 
#include <stdio.h>
#include <stdarg.h>
#include <string.h>


#define PACKET_HEAD     0xA5
#define PACKET_TAIL     0x5A
#define FLOAT_BYTE_LEN  4
#define MAX_FLOAT_CNT   10
#define INT8_BYTE_LEN    1
#define MAX_INT8_CNT    32
#define BLE_RX_BUF_LEN   128


typedef enum 
    {
	    BLE_MODE_PACKET = 0,
	    BLE_MODE_VOFA        
    } BLE_Mode;

void BLE_StartRx(void); 
void BLE_SetMode(BLE_Mode mode);
void BLE_SendByte(uint8_t Byte);
void BLE_SendArray(uint8_t *Array, uint16_t Length);
void BLE_SendString(char *String);
void BLE_Printf(char *format, ...);
int8_t BLE_SendFloatArrayPacket(float *float_array, uint8_t array_len);
int8_t BLE_RecvFloatArrayPacket(float *float_array);
int8_t BLE_SendInt8ArrayPacket(int8_t *int8_array, uint8_t array_len);
int8_t BLE_RecvInt8ArrayPacket(int8_t *int8_array);

#endif
