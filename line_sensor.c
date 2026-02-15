//******************************************************************************
//  * @file           : line_sensor.c
//  ******************************************************************************

#include "line_sensor.h"
#include "I2C_soft.h"

#define LINE_SENSOR_ADDR_7BIT 0x3C
#define LINE_SENSOR_ADDR_W    (LINE_SENSOR_ADDR_7BIT << 1)
#define LINE_SENSOR_ADDR_R    ((LINE_SENSOR_ADDR_7BIT << 1) | 0x01)

#define LINE_SENSOR_REG_RAW   0x00
#define LINE_SENSOR_REG_OFF   0x04

static uint8_t line_raw = 0;
static int8_t line_offset = 0;

static uint8_t LineSensor_ReadReg(uint8_t reg)
{
    uint8_t data = 0;

    MyI2C_Start();
    MyI2C_SendByte(LINE_SENSOR_ADDR_W);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(LINE_SENSOR_ADDR_R);
    MyI2C_ReceiveAck();
    data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return data;
}

void LineSensor_Init(void)
{
    // No device-specific init required for default settings.
}

void LineSensor_Update(void)
{
    line_raw = LineSensor_ReadReg(LINE_SENSOR_REG_RAW);
    line_offset = (int8_t)LineSensor_ReadReg(LINE_SENSOR_REG_OFF);
}

uint8_t LineSensor_GetRaw(void)
{
    return line_raw;
}

int8_t LineSensor_GetOffset(void)
{
    return line_offset;
}
