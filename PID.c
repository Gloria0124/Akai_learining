//******************************************************************************
//  * @file           : PID.c
//  ******************************************************************************

#include "main.h"
#include "PID.h"
#include <math.h>
//防止过饱和
static inline float clampf(float v, float lo, float hi)
{
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

void PID_Cal(PID_TypeDef *P)
{


  const float target = P->Target;
  const float actual = P->Actual_new;
  const float error  = target - actual;
  const float Kp = P->Kp;
  const float Ki = P->Ki;
  const float Kd = P->Kd;
  const float K  = P->K; 

  //抗积分饱和增量
  float increment = error / (K * fabsf(error) + 1.0f);
  P->Error_add += increment;
  P->Error_add = clampf(P->Error_add, P->MinIntegral, P->MaxIntegral);
	//p
  const float p_out = Kp * error;
	//I
  const float i_out = (Ki != 0.0f) ? (Ki * P->Error_add) : 0.0f;
	//d
  float d_out = 0.0f;
  if (Kd != 0.0f) 
	{
    float raw_d_out = -Kd * (actual - P->Actual_old);
		d_out = 0.2f * raw_d_out + 0.8f * P->Last_Dout;
	
    P->Last_Dout = d_out;
  }

  // 输出总和
  float total_out = p_out + i_out + d_out;
	total_out = clampf(total_out, P->MinOut, P->MaxOut);

 //更新
  P->POut       = p_out;
  P->IOut       = i_out;
  P->DOut       = d_out;
  P->Out        = total_out;
  P->Error_new  = error;
  P->Actual_old = actual;
}
