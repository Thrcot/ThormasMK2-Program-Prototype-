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

#endif /* INC_BMX055_HAL_H_ */
