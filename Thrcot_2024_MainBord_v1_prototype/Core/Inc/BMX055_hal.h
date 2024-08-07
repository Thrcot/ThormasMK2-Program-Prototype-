/*
 * BMX055_hal.h
 *
 *  Created on: Aug 7, 2024
 *      Author: neoki
 */

#ifndef INC_BMX055_HAL_H_
#define INC_BMX055_HAL_H_

#include "stm32f4xx_hal.h"

#define ACCEL_ADR (0x19 << 1)
#define GYRO_ADR (0x69 << 1)
#define MAG_ADR (0x13 << 1)

extern I2C_HandleTypeDef *IMU_i2c;

int BMX055_Init(I2C_HandleTypeDef *hi2c);

double Gyro_Get_X(void);
double Gyro_Get_Y(void);
double Gyro_Get_Z(void);

#endif /* INC_BMX055_HAL_H_ */
