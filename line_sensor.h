#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#include <stdint.h>

void LineSensor_Init(void);
void LineSensor_Update(void);
uint8_t LineSensor_GetRaw(void);
int8_t LineSensor_GetOffset(void);

#endif
