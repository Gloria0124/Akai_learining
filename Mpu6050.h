#ifndef __I2C_RJ_H
#define __I2C_RJ_H

#include "stdint.h"

//寄存器
#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C
#define	MPU6050_CONFIG				0x1A

//偏移
extern float MPU6050_GYRO_OFFSET_X;
#define MPU6050_GYRO_OFFSET_Y  0
#define MPU6050_GYRO_OFFSET_Z  0
#define MPU6050_ACCEL_OFFSET_X 0
#define MPU6050_ACCEL_OFFSET_Y 0
#define MPU6050_ACCEL_OFFSET_Z 0

//转化系数
#define ACCEL_SCALE_FACTOR (4.0f / 32768.0f) 
#define GYRO_SCALE_FACTOR  (250.0f / 32768.0f) 

typedef struct {
    int16_t AccX;  
    int16_t AccY;  
    int16_t AccZ;  
    int16_t GyroX; 
    int16_t GyroY; 
    int16_t GyroZ; 
    
    float AccX_Scaled; 
    float AccY_Scaled; 
    float AccZ_Scaled; 
    float GyroX_Scaled;
    float GyroY_Scaled;
    float GyroZ_Scaled;
    
    uint8_t DeviceID; 
    uint8_t IsInitOK; 
	
		float Roll;  
    float Pitch; 
    float Yaw;   
} MPU6050_DevTypeDef;

// 初始化MPU6050
void MPU6050_Init(void);

// 读取原始数据 
void MPU6050_ReadRawData(MPU6050_DevTypeDef *dev);

//转化原数据
void MPU6050_CalcScaledData(MPU6050_DevTypeDef *dev);
void MPU6050_ComplementaryFilter(MPU6050_DevTypeDef *dev, uint16_t dt_ms, float DifSpeed);


uint8_t IIC_ReadReg(uint8_t reg);

#endif

