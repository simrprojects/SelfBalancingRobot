#ifndef OSI2C_H_
#define OSI2C_H_

#include "stm32f7xx_hal.h"

typedef void* I2CComPortHandle;

int  OSI2C_Init(I2CComPortHandle * scp,I2C_HandleTypeDef* hi2c);
int OSI2C_Mem1Read(I2CComPortHandle h,unsigned char devAddress,unsigned char memAddress,void* data,int size,int timeout);
int OSI2C_Mem1Write(I2CComPortHandle h,unsigned char devAddress,unsigned char memAddress,void* data,int size,int timeout);


#endif
