#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;                  	
    float Ki;                  
    float Kd;				
		float K;					//变速积分系数
	
		float POut;
		float IOut;
		float DOut;
		float Last_Dout;
    // 限幅
    float MaxOut;              	
    float MinOut;              	
    float MaxIntegral;         	// 积分项最大值
    float MinIntegral;         	// 积分项最小值
    
    float Target;  
    float Actual_new;  
		float Actual_old;
    float Error_new;  
    float Error_old;  
    float Error_add;
    float Out;     
} PID_TypeDef;

void PID_Cal(PID_TypeDef *P);

#endif
