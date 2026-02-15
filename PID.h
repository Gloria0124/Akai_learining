#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;                  	
    float Ki;                  
    float Kd;				
		float K;					
	
		float POut;
		float IOut;
		float DOut;
		float Last_Dout;
  
    float MaxOut;              	
    float MinOut;              	
    float MaxIntegral;        
    float MinIntegral;         
    
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
