//******************************************************************************
//  * @file           : Mpu6050.c
//  ******************************************************************************

#include "Mpu6050.h"   
#include "I2C_soft.h" 
#include "math.h"

#define SlaveAddress 0x68 
#define RegAddress 0x3B    

// 写入寄存器函数
void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(SlaveAddress << 1); // 写地址
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);               // 寄存器地址
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);              // 数据
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

// 读取多个寄存器数据
void MPU6050_ReadMultiReg(uint8_t reg, uint8_t *data, uint8_t count)
{
    MyI2C_Start();
    MyI2C_SendByte(SlaveAddress << 1); // 写地址
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);               // 寄存器地址
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte((SlaveAddress << 1) | 0x01); // 读地址
    MyI2C_ReceiveAck();

    for(uint8_t i = 0; i < count; i++) {
        data[i] = MyI2C_ReceiveByte();
        // 最后一个字节发送NACK(1)，其他发送ACK(0)
        MyI2C_SendAck(i == (count - 1) ? 1 : 0);
    }
    
    MyI2C_Stop();
}

// 读取单个寄存器
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t data;
    MPU6050_ReadMultiReg(reg, &data, 1);
    return data;
}

// MPU6050初始化函数
void MPU6050_Init(void)
{
  MyI2C_Init(); 

 	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x07);
	MPU6050_WriteReg(MPU6050_CONFIG,0x01);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x00);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x08);
}

// 读取传感器原始数据
void MPU6050_ReadRawData(MPU6050_DevTypeDef *dev)
{
  uint8_t buffer[14];
  
  // 从0x3B开始读取14个字节
  MPU6050_ReadMultiReg(0x3B, buffer, 14);
  
  // 组合加速度计数据 (高8位<<8 | 低8位)
  dev->AccX = (int16_t)((buffer[0] << 8) | buffer[1]); 
  dev->AccY = (int16_t)((buffer[2] << 8) | buffer[3]); 
  dev->AccZ = (int16_t)((buffer[4] << 8) | buffer[5]); 
  
  // 组合陀螺仪数据
  dev->GyroX = (int16_t)((buffer[8] << 8) | buffer[9]);  
  dev->GyroY  = (int16_t)((buffer[10] << 8) | buffer[11]); 
  dev->GyroZ = (int16_t)((buffer[12] << 8) | buffer[13]); 
	
	MPU6050_CalcScaledData(dev);
}

void MPU6050_CalcScaledData(MPU6050_DevTypeDef *dev)
{
	//加速度
	dev->AccX_Scaled = (dev->AccX - MPU6050_ACCEL_OFFSET_X) * ACCEL_SCALE_FACTOR;
	dev->AccY_Scaled = (dev->AccY - MPU6050_ACCEL_OFFSET_Y) * ACCEL_SCALE_FACTOR;
	dev->AccZ_Scaled = (dev->AccZ - MPU6050_ACCEL_OFFSET_Z) * ACCEL_SCALE_FACTOR;
	
	//陀螺仪
	dev->GyroX_Scaled = (dev->GyroX - MPU6050_GYRO_OFFSET_X) * GYRO_SCALE_FACTOR;
	dev->GyroY_Scaled = (dev->GyroY - MPU6050_GYRO_OFFSET_Y) * GYRO_SCALE_FACTOR;
	dev->GyroZ_Scaled = (dev->GyroZ - MPU6050_GYRO_OFFSET_Z) * GYRO_SCALE_FACTOR;
}

void MPU6050_ComplementaryFilter(MPU6050_DevTypeDef *dev, uint16_t dt_ms, float DifSpeed)
{
    float dt = (float)dt_ms / 1000.0f;

    //加速度
    float acc_xz = sqrt(dev->AccX_Scaled*dev->AccX_Scaled + dev->AccZ_Scaled*dev->AccZ_Scaled) + 1e-6f;
    float acc_roll = atan2(dev->AccY_Scaled, acc_xz) * 57.2957795131f;
    
    float acc_yz = sqrt(dev->AccY_Scaled*dev->AccY_Scaled + dev->AccZ_Scaled*dev->AccZ_Scaled) + 1e-6f;
    float acc_pitch = atan2(-dev->AccX_Scaled, acc_yz) * 57.2957795131f;
    
    //陀螺仪
    dev->Roll += dev->GyroX_Scaled * dt;
    dev->Pitch += dev->GyroY_Scaled * dt;
	
	float base_alpha = 0.02f;
	float reduction = fabs(DifSpeed)*0.0001f;
	float alpha = base_alpha - reduction;
	if(alpha < 0.001f)
		alpha = 0.001f;
	
	dev->Pitch = alpha * acc_pitch + (1 - alpha) * dev->Pitch;
	dev->Roll = alpha * acc_roll + (1 - alpha) * dev->Roll;

}
