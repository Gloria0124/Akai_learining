#include "Mpu6050.h"   
#include "I2C_soft.h" 

#define SlaveAddress 0x68 
#define RegAddress 0x3B    

// 写入寄存器函数
void IIC_WriteReg(uint8_t reg, uint8_t data)
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
void IIC_ReadMultiReg(uint8_t reg, uint8_t *data, uint8_t count)
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
uint8_t IIC_ReadReg(uint8_t reg)
{
    uint8_t data;
    IIC_ReadMultiReg(reg, &data, 1);
    return data;
}

// MPU6050初始化函数
void MPU6050_Init(void)
{
    MyI2C_Init(); // 建议在这里确保底层I2C被初始化，防止main函数忘记调用

    // 1. 复位电源管理寄存器 (唤醒MPU6050)
    IIC_WriteReg(0x6B, 0x00);
    HAL_Delay(50); // 唤醒后稍作延时
    
    // 2. 设置陀螺仪采样率 (可选，默认亦可)
    IIC_WriteReg(0x19, 0x07); 

    // 3. 设置加速度计量程 ±2g
    IIC_WriteReg(0x1C, 0x00);
    
    // 4. 设置陀螺仪量程 
    IIC_WriteReg(0x1B, 0x18); 
    
    // 5. 再次写0x6B确保唤醒并选择时钟源
    IIC_WriteReg(0x6B, 0x01);
	
		IIC_WriteReg(0x38,0x01);
}

// 读取传感器原始数据
void MPU6050_ReadRawData(int16_t *accel, int16_t *gyro)
{
    uint8_t buffer[14];
    
    // 从0x3B开始读取14个字节
    IIC_ReadMultiReg(0x3B, buffer, 14);
    
    // 组合加速度计数据 (高8位<<8 | 低8位)
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]); 
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]); 
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]); 
    
    // 组合陀螺仪数据
    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);  
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]); 
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]); 
}

