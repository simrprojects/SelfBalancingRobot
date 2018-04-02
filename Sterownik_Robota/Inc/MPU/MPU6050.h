/*
 * MPU6050.h
 *
 *  Created on: 23.03.2018
 *      Author: Przemek
 */

#ifndef MPU_MPU6050_H_
#define MPU_MPU6050_H_

/**\typedef struct tMPUMeasurement
 * \brief definicja typu do przenoszenia rekordu pomiarowego
 */
typedef struct{
	int timestamp;
	float omega[3];
	float acceleration[3];
	float rpy[3];
}tMPUMeasuremenet;

typedef struct{
	unsigned char i2cAddress;
}tMPUHardwareSetting;

typedef struct{
	int queueDepth;
}tMPUConfiguration;
/**\typedef void* tMPUHandler
 *\brief
 */
typedef void* tMPUHandler;

int MPU6050_Init(tMPUHandler *handler,tMPUHardwareSetting *hwSettings,tMPUConfiguration *config);
int MPU6050_UpdateFromISR(tMPUHandler h, unsigned int timestamp);
int MPU6050_GetMeasurement(tMPUHandler h,tMPUMeasuremenet *measuremenet,int timeout);

#endif /* MPU_MPU6050_H_ */
