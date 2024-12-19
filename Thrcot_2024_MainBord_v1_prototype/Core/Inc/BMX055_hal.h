/*
 * BMX055_hal.h
 *
 *  Created on: Aug 7, 2024
 *      Author: neoki
 */

#ifndef INC_BMX055_HAL_H_
#define INC_BMX055_HAL_H_

#include <math.h>
#include "stm32f4xx_hal.h"

#define ACCEL_ADR (0x19 << 1)
#define GYRO_ADR (0x69 << 1)
#define MAG_ADR (0x13 << 1)

extern I2C_HandleTypeDef *IMU_i2c;

extern int Accel_Change;
extern int Gyro_Change;

int BMX055_Init(I2C_HandleTypeDef *hi2c, int AccelRange, int GyroRange);

int Accel_Get_X(void);
int Accel_Get_Y(void);
int Accel_Get_Z(void);

int Gyro_Get_X(void);
int Gyro_Get_Y(void);
int Gyro_Get_Z(void);

double Get_Yaw(int offset, double duration);

int Gyro_Offset_Z(int tryCount);

#endif /* INC_BMX055_HAL_H_ */
