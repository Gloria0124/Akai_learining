#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

extern int flag;
extern float Encoder_Left ;
extern float Encoder_Right ;
extern  float Target_Pitch;
extern int PWM_Out;
extern float Mechanical_Angle;
extern int Turn_Target;
void Get_Angle_Complementary(void);
extern float AveSpeed;
extern float PWML, PWMR;
extern float Target_Speed_Raw; 
extern float Target_Turn_Raw;  
#endif
