/*
 * BMX055_hal.c
 *
 *  Created on: Aug 7, 2024
 *      Author: neoki
 */

#include "BMX055_hal.h"

I2C_HandleTypeDef *IMU_i2c;

int BMX055_Init(I2C_HandleTypeDef *hi2c)
{
	IMU_i2c = hi2c;

	int Error_Message = 0;
	uint8_t Accel_Init_buf[] = {0x0F, 0x03, 0x10, 0x08, 0x11, 0x00};
	uint8_t Gyro_Init_buf[] = {0x0F, 0x04, 0x10, 0x07, 0x11, 0x00};
	uint8_t Mag_Init_buf[] = {0x4B, 0x83, 0x4B, 0x01, 0x4C, 0x00, 0x4E, 0x84, 0x51, 0x04, 0x52, 0x16};
	uint8_t Check_Transmit1[] = {0x00};
	uint8_t Check_Transmit2[] = {0x00};
	uint8_t Check_Transmit3[] = {0x40};
	uint8_t Receive_Data[10];

	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Accel_Init_buf, sizeof(Accel_Init_buf), 1000);
	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Gyro_Init_buf, sizeof(Gyro_Init_buf), 1000);
	HAL_I2C_Master_Transmit(IMU_i2c, MAG_ADR, Mag_Init_buf, sizeof(Mag_Init_buf), 1000);

	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Check_Transmit1, 1, 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Receive_Data, 1, 1000);
	if (Receive_Data[0] != 0xFA) {
		Error_Message |= 1;
	}

	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Check_Transmit2, 1, 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Receive_Data, 1, 1000);
	if (Receive_Data[0] != 0x0F) {
		Error_Message |= 2;
	}

	HAL_I2C_Master_Transmit(IMU_i2c, MAG_ADR, Check_Transmit3, 1, 1000);
	HAL_I2C_Master_Receive(IMU_i2c, MAG_ADR, Receive_Data, 1, 1000);
	if (Receive_Data[0] != 0x32) {
		Error_Message |= 4;
	}

	return Error_Message;
}
