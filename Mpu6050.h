#ifndef __I2C_RJ_H
#define __I2C_RJ_H

#include "stdint.h"


// 初始化MPU6050
void MPU6050_Init(void);

// 读取原始数据 (加速度计 + 陀螺仪)
void MPU6050_ReadRawData(int16_t *accel, int16_t *gyro);

// 如果偶尔需要单独读取某个寄存器
uint8_t IIC_ReadReg(uint8_t reg);

#endif

