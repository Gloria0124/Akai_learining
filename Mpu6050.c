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

// 卡尔曼滤波初始化
void Kalman_Init(KalmanFilter_TypeDef *kalman, float q, float r)
{
    kalman->q = q;  // 过程噪声方差（陀螺仪噪声）
    kalman->r = r;  // 测量噪声方差（加速度计噪声）
    kalman->x = 0.0f;  // 初始状态
    kalman->p = 1.0f;  // 初始估计误差
    kalman->k = 0.0f;  // 初始卡尔曼增益
}

// 卡尔曼滤波计算
float Kalman_Filter(KalmanFilter_TypeDef *kalman, float z, float dt)
{
    // 预测阶段
    kalman->p = kalman->p + kalman->q;
    
    // 更新阶段 - 计算卡尔曼增益
    kalman->k = kalman->p / (kalman->p + kalman->r);
    
    // 状态更新
    kalman->x = kalman->x + kalman->k * (z - kalman->x);
    
    // 误差协方差更新
    kalman->p = (1.0f - kalman->k) * kalman->p;
    
    return kalman->x;
}

// 改进的卡尔曼滤波IMU融合
void MPU6050_KalmanFilter(MPU6050_DevTypeDef *dev, uint16_t dt_ms, float DifSpeed)
{
    float dt = (float)dt_ms / 1000.0f;
    
    // 初始化卡尔曼滤波器（第一次调用）
    static uint8_t init_flag = 0;
    if(init_flag == 0)
    {
        // q越大，越信任陀螺仪；r越小，越信任加速度计
        Kalman_Init(&dev->Kalman_Roll, 0.01f, 5.0f);  
        Kalman_Init(&dev->Kalman_Pitch, 0.01f, 5.0f);  
        init_flag = 1;
    }
    
    // 从加速度计计算角度
    float acc_xz = sqrt(dev->AccX_Scaled*dev->AccX_Scaled + dev->AccZ_Scaled*dev->AccZ_Scaled) + 1e-6f;
    float acc_roll = atan2(dev->AccY_Scaled, acc_xz) * 57.2957795131f;
    
    float acc_yz = sqrt(dev->AccY_Scaled*dev->AccY_Scaled + dev->AccZ_Scaled*dev->AccZ_Scaled) + 1e-6f;
    float acc_pitch = atan2(-dev->AccX_Scaled, acc_yz) * 57.2957795131f;
    
    // 根据速度差动态调整信任度
    float speed_factor = 1.0f + fabs(DifSpeed) * 0.001f;
    dev->Kalman_Roll.q = 0.01f * speed_factor;
    dev->Kalman_Pitch.q = 0.01f * speed_factor;
    
    // 卡尔曼滤波融合
    dev->Roll = Kalman_Filter(&dev->Kalman_Roll, acc_roll, dt);
    dev->Roll += dev->GyroX_Scaled * dt * 0.98f;  // 添加陀螺仪补偿
    
    dev->Pitch = Kalman_Filter(&dev->Kalman_Pitch, acc_pitch, dt);
    dev->Pitch += dev->GyroY_Scaled * dt * 0.98f;  // 添加陀螺仪补偿
}
