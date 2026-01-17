#include "control.h"
#include "main.h" // 包含引脚定义
#include "math.h" // 数学库，用于atan2计算
#include "Mpu6050.h"
#include "motor_pwm.h"
#include "encoder.h"

// --- 全局变量 ---
float Pitch, Roll, Yaw;            // 角度
float Gyro_Y, Gyro_X, Gyro_Z;      // 角速度
int16_t Accel_Raw[3], Gyro_Raw[3]; // 原始数据

// 平衡参数
float Mechanical_Angle = 1.50f; //
float K_Angle = 0.14f;          // 互补滤波系数 (越小越相信陀螺仪，越大越相信加速度计)

// PID全局参数

// 直立环控制
float Vertical_Kp = 450.0f; // 比例系数：决定回复力的强弱
float Vertical_Kd = 20.0f;  // 微分系数：决定阻尼力

// 速度环参数 (PI控制)
float Velocity_Kp = 0; // 比例系数 0.5-10
float Velocity_Ki = 0; // 积分系数 0.002-0.05

// 过程变量
float Balance_Out = 0;
float Velocity_Out = 0;
int PWM_Out = 0;

// --- 直立环PD ---

// Gyro 当前车身倒下的角速度
float Vertical_PID(float Angle, float Gyro)
{
    float Bias;
    float pwm_out;

    Bias = Angle - Mechanical_Angle;

    // 直立环核心公式：Kp*角度 + Kd*角速度
    pwm_out = Vertical_Kp * Bias + Vertical_Kd * Gyro;

    return pwm_out;
}

static float Velocity_Integral = 0;   // 速度积分 （如果小车以很大的速度往前漂移，此积分积累起来后，会强迫小车退回来）
static float Velocity_Filter_Old = 0; // 滤波旧值，使电机更丝滑
// --- 速度环PI---

float Velocity_PID(int Encoder_Left, int Encoder_Right)
{

    float Velocity_Error;
    float Velocity_Filter_New;

    // 计算总速度

    int Current_Speed = Encoder_Left + Encoder_Right;

    // 2简单低通滤波
    // 0.3是新数据权重，0.7是旧数据权重
    Velocity_Filter_New = 0.7f * Velocity_Filter_Old + 0.3f * Current_Speed;
    Velocity_Filter_Old = Velocity_Filter_New;

    // 偏差计算 (目标速度是0)
    Velocity_Error = Velocity_Filter_New - 0;

    // 积分累加，
    Velocity_Integral += Velocity_Error;

    // 5. 积分限幅 (防止被拿起来空转时积分过大)
    if (Velocity_Integral > 10000)
        Velocity_Integral = 10000;
    if (Velocity_Integral < -10000)
        Velocity_Integral = -10000;

    // 6. 速度环核心公式
    return Velocity_Kp * Velocity_Error + Velocity_Ki * Velocity_Integral;
}

void PID_Reset(void)
{

    Velocity_Integral = 0;
    Velocity_Filter_Old = 0;
    Balance_Out = 0;
    Velocity_Out = 0;
    PWM_Out = 0;
}

// --- 互补滤波计算角度 ---
void Get_Angle_Complementary(void)
{
    // 1. 读取原始数据
    MPU6050_ReadRawData(Accel_Raw, Gyro_Raw);

    // 2. 计算加速度计得到的角度
    // 57.3 = 180/PI
    float Accel_Angle = atan2(Accel_Raw[1], Accel_Raw[2]) * 57.296f;

    // 3. 得到陀螺仪角速度 (假设量程±2000，灵敏度16.4 LSB/(deg/s))

    Gyro_Y = Gyro_Raw[1] / 16.4f;

    // 4. 互补滤波核心公式
    // 当前角度 = (0.98 * (上次角度 + 角速度*dt)) + (0.02 * 加速度角度)
    // dt = 0.01s (10ms中断一次)
    Pitch = (1 - K_Angle) * (Pitch + Gyro_Y * 0.008f) + K_Angle * Accel_Angle;
}

// 直立环为主力，时刻都在调整，速度环为辅，改变车身的倾角重心
/*

Vertical_Kp	直立P	回正的硬度	太小：扶不起来；太大：疯狂高频抖动
Vertical_Kd	直立D	减震器的阻尼	太小：车晃来晃去停不稳；太大：车反应迟钝
Velocity_Kp	速度P	刹车的灵敏度	太小：车扶起来后会一直加速跑走；太大：车会前后猛晃
Velocity_Ki	速度I	，控制超调，太小：车会停不住，一直缓慢漂移；太大：车会来回大幅度摆动

*/

// --- 中断标志位 ---
volatile uint8_t MPU6050_DataReady_Flag = 0; // MPU6050数据准备好标志位

// --- 外部中断回调函数 (仅设置标志位，快速退出) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        // 只翻转LED指示中断进入，设置标志位
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        MPU6050_DataReady_Flag = 1; // 设置标志位，主循环会处理
    }
}

// --- 平衡控制主处理函数 (在主循环中调用) ---
void Balance_Control_Process(void)
{
    // 检查标志位
    if (MPU6050_DataReady_Flag == 0)
    {
        return; // 数据未准备好，直接返回
    }

    // 清除标志位
    MPU6050_DataReady_Flag = 0;

    // 获取姿态角度
    Get_Angle_Complementary();

    // 获取编码器速度
    int Encoder_Left = Read_Speed(2);
    int Encoder_Right = Read_Speed(4);

    // 直立环 PID 计算 (输入：角度Pitch，角速度Gyro_Y)
    Balance_Out = Vertical_PID(Pitch, Gyro_Y);

    // 速度环 PID 计算 (输入：编码器速度)
    Velocity_Out = Velocity_PID(Encoder_Left, Encoder_Right);

    // 最终PWM计算
    PWM_Out = (int)(Balance_Out + Velocity_Out);

    if (PWM_Out > 0)
        PWM_Out += 800; // 向上推过死区
    else if (PWM_Out < 0)
        PWM_Out -= 800; // 向下推过死区

    // --- 重新放回：总限幅 ---
    if (PWM_Out > 4500)
        PWM_Out = 4500;
    if (PWM_Out < -4500)
        PWM_Out = -4500;

    // 倒地保护
    if (Pitch < -35 || Pitch > 35)
    {
        PWM_Out = 0;
        PID_Reset();     // 重置积分等状态
        Motor_Set(0, 0); // 强制停机
    }
    else
    {
        // 正常输出给电机
        Motor_Set(PWM_Out, PWM_Out);
    }
}
