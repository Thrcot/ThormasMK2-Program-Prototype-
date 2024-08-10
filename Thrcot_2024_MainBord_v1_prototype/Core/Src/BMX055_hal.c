/*
 * BMX055_hal.c
 *
 *  Created on: Aug 7, 2024
 *      Author: neoki
 */

#include "BMX055_hal.h"

I2C_HandleTypeDef *IMU_i2c;

int BMX055_Init(I2C_HandleTypeDef *hi2c, int AccelRange, int GyroRange)
{
	IMU_i2c = hi2c;

	int Error_Message = 0;
	uint8_t Accel_Init_buf[] = {0x0F, 0x03, 0x10, 0x08, 0x11, 0x00};
	uint8_t Gyro_Init_buf[] = {0x0F, 0x01, 0x10, 0x07, 0x11, 0x00};
	uint8_t Mag_Init_buf[] = {0x4B, 0x83, 0x4B, 0x01, 0x4C, 0x00, 0x4E, 0x84, 0x51, 0x04, 0x52, 0x16};
	uint8_t Check_Transmit1[] = {0x00};
	uint8_t Check_Transmit2[] = {0x00};
	uint8_t Check_Transmit3[] = {0x40};
	uint8_t Receive_Data[10];

	switch (AccelRange) {
		case 2:
			Accel_Init_buf[1] = 0x03;
			break;

		case 4:
			Accel_Init_buf[1] = 0x05;
			break;

		case 8:
			Accel_Init_buf[1] = 0x08;
			break;

		case 16:
			Accel_Init_buf[1] = 0x0C;
			break;
		default:
			break;
	}

	switch (GyroRange) {
		case 125:
			Gyro_Init_buf[1] = 0x04;
			break;

		case 250:
			Gyro_Init_buf[1] = 0x03;
			break;

		case 500:
			Gyro_Init_buf[1] = 0x02;
			break;

		case 1000:
			Gyro_Init_buf[1] = 0x01;
			break;

		case 2000:
			Gyro_Init_buf[1] = 0x00;

		default:
			break;
	}

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

int Accel_Get_X(void)
{
	uint8_t Data_buf[10];
	uint8_t Request_Data_buf[] = {0x02};
	int LSB_Data = 0;
	int MSB_Data = 0;
	int Accel_X;

	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Data_buf, 1, 1000);
	LSB_Data = Data_buf[0];

	Request_Data_buf[0] = 0x03;
	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Data_buf, 1, 1000);
	MSB_Data = Data_buf[0];

	Accel_X = (MSB_Data << 4) | (LSB_Data >> 4);
	if (Accel_X > 2047) {
		Accel_X -= 4095;
	}

	return Accel_X;
}

int Accel_Get_Y(void)
{
	uint8_t Data_buf[10];
	uint8_t Request_Data_buf[] = {0x04};
	int LSB_Data = 0;
	int MSB_Data = 0;
	int Accel_Y;

	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Data_buf, 1, 1000);
	LSB_Data = Data_buf[0];

	Request_Data_buf[0] = 0x05;
	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Data_buf, 1, 1000);
	MSB_Data = Data_buf[0];

	Accel_Y = (MSB_Data << 4) | (LSB_Data >> 4);
	if (Accel_Y > 2047) {
		Accel_Y -= 4095;
	}

	return Accel_Y;
}

int Accel_Get_Z(void)
{
	uint8_t Data_buf[10];
	uint8_t Request_Data_buf[] = {0x06};
	int LSB_Data = 0;
	int MSB_Data = 0;
	int Accel_Z;

	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Data_buf, 1, 1000);
	LSB_Data = Data_buf[0];

	Request_Data_buf[0] = 0x07;
	HAL_I2C_Master_Transmit(IMU_i2c, ACCEL_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, ACCEL_ADR, Data_buf, 1, 1000);
	MSB_Data = Data_buf[0];

	Accel_Z = (MSB_Data << 4) | (LSB_Data >> 4);
	if (Accel_Z > 2047) {
		Accel_Z -= 4095;
	}

	return Accel_Z;
}

int Gyro_Get_X(void)
{
	uint8_t Data_buf[10];
	uint8_t Request_Data_buf[] = {0x02};
	int LSB_Data = 0;
	int MSB_Data = 0;
	int Gyro_X;

	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Data_buf, 1, 1000);
	LSB_Data = Data_buf[0];

	Request_Data_buf[0] = 0x03;
	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Data_buf, 1, 1000);
	MSB_Data = Data_buf[0];

	Gyro_X = (MSB_Data << 8) + LSB_Data;
	if (Gyro_X > 32767) {
		Gyro_X -= 65535;
	}

	return Gyro_X;
}

int Gyro_Get_Y(void)
{
	uint8_t Data_buf[10];
	uint8_t Request_Data_buf[] = {0x04};
	int LSB_Data = 0;
	int MSB_Data = 0;
	int Gyro_Y;

	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Data_buf, 1, 1000);
	LSB_Data = Data_buf[0];

	Request_Data_buf[0] = 0x05;
	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Data_buf, 1, 1000);
	MSB_Data = Data_buf[0];

	Gyro_Y = (MSB_Data << 8) + LSB_Data;
	if (Gyro_Y > 32767) {
		Gyro_Y -= 65535;
	}

	return Gyro_Y;
}

int Gyro_Get_Z(void)
{
	uint8_t Data_buf[10];
	uint8_t Request_Data_buf[] = {0x06};
	int LSB_Data = 0;
	int MSB_Data = 0;
	int Gyro_Z;

	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Data_buf, 1, 1000);
	LSB_Data = Data_buf[0];

	Request_Data_buf[0] = 0x07;
	HAL_I2C_Master_Transmit(IMU_i2c, GYRO_ADR, Request_Data_buf, sizeof(Request_Data_buf), 1000);
	HAL_I2C_Master_Receive(IMU_i2c, GYRO_ADR, Data_buf, 1, 1000);
	MSB_Data = Data_buf[0];

	Gyro_Z = (MSB_Data << 8) + LSB_Data;
	if (Gyro_Z > 32767) {
		Gyro_Z -= 65535;
	}

	return Gyro_Z;
}

int Gyro_Offset_Z(int tryCount)
{
	int Offset = 0.0;

	for (int i = 0; i < tryCount; i++) {
		Offset += Gyro_Get_Z();
	}

	return Offset / tryCount;
}
