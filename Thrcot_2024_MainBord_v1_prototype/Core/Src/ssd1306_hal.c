/*
 * ssd1306_hal.c
 *
 *  Created on: Jul 29, 2024
 *      Author: neoki
 */

#include "ssd1306_hal.h"

uint8_t OLED_Init_Data[30] = {
		0x00,
		0xA8, 0x3F,
		0xD3, 0x00,
		0x40,
		0xA1,
		0xC8,
		0xDA, 0x12,
		0x81, 0xFF,
		0xA4,
		0xA6,
		0xD5, 0x80,
		0x20, 0x00,
		0x8D, 0x14,
		0xAF
};

uint8_t All_Display_Data[8][129];

long map(long x, long min_in, long max_in, long min_out, long max_out)
{
	return (x - min_in) * (max_out - min_out) / (max_in - min_out) + min_out;
}

uint8_t Changing_Y_page(int Y)
{
	return Y / 8;
}

uint8_t Changing_Y_val(int Y)
{
	return 0x01 << (Y % 8);
}

void OLED_Init(I2C_HandleTypeDef *hi2c, uint8_t *Init_data, int size)
{
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Init_data, size, HAL_MAX_DELAY);
	OLED_AllClear(hi2c);
}

void OLED_Display(I2C_HandleTypeDef *hi2c)
{
	uint8_t Display_Range[] = {0x00, 0x22, 0, 7, 0x21, 0, 127};

	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Display_Range, sizeof(Display_Range), HAL_MAX_DELAY);

	for (int i = 0; i < 8; i++) {
			HAL_I2C_Master_Transmit(hi2c, OLED_ADR, All_Display_Data[i], sizeof(All_Display_Data[i]), HAL_MAX_DELAY);
	}
}

void OLED_Thrcot_Logo_Display(I2C_HandleTypeDef *hi2c)
{
	uint8_t Logo_Range_Data[] = {0x00, 0x22, 2, 5, 0x21, 32, 95};

	uint8_t Thrcot_Logo_Data1[65] = {
			0x40,
			0x00, 0x00, 0x00, 0x20, 0x30, 0x38, 0x38, 0x38,
			0x38, 0xF8, 0x38, 0x38, 0x38, 0x38, 0x18, 0x08,
			0x00, 0xF8, 0x80, 0xC0, 0xC0, 0xC0, 0x80, 0x00,
			0x00, 0xC0, 0x30, 0xCC, 0x82, 0xC0, 0xC0, 0x40,
			0x00, 0x00, 0x80, 0xC0, 0xC0, 0xC0, 0x80, 0x00,
			0x00, 0x00, 0x00, 0xC0, 0x80, 0xE0, 0xE0, 0x80,
			0xC0, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xF0, 0xC0,
			0xC0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Logo_Data2[65] = {
			0x40,
			0x00, 0x00, 0x00, 0xE0, 0x10, 0x10, 0x18, 0x00,
			0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0xFF, 0xF0,
			0x0E, 0x01, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00,
			0x1E, 0x3F, 0x61, 0xC0, 0xC0, 0xC0, 0x61, 0x33,
			0x00, 0x1B, 0x1F, 0x7E, 0x31, 0xE0, 0xE0, 0x31,
			0x7E, 0x1F, 0x1B, 0x00, 0x00, 0x00, 0x7F, 0xC0,
			0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Logo_Data3[65] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08,
			0x08, 0x18, 0x10, 0x30, 0x20, 0x60, 0x40, 0x40,
			0x40, 0x80, 0x80, 0x80, 0xE0, 0xBE, 0x03, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0xFC, 0xF4, 0xFC, 0xF4, 0xFC,
			0xF0, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0xFC, 0xF0,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Logo_Data4[65] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0xF8, 0x0F, 0x01, 0x01, 0x01,
			0x01, 0x03, 0x02, 0x02, 0x02, 0x02, 0x06, 0x04,
			0x04, 0x04, 0x04, 0x04, 0x04, 0x0C, 0x08, 0x08,
			0x08, 0x08, 0x08, 0x08, 0x19, 0x19, 0x10, 0x10,
			0x11, 0x10, 0x10, 0x11, 0x30, 0x30, 0x21, 0x20,
			0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Logo_Range_Data, sizeof(Logo_Range_Data), HAL_MAX_DELAY);

	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Logo_Data1, sizeof(Thrcot_Logo_Data1), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Logo_Data2, sizeof(Thrcot_Logo_Data2), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Logo_Data3, sizeof(Thrcot_Logo_Data3), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Logo_Data4, sizeof(Thrcot_Logo_Data4), HAL_MAX_DELAY);
}

void OLED_Thrcot_Large_Logo_Display(I2C_HandleTypeDef *hi2c)
{
	uint8_t Large_Logo_Range[] = {0x00, 0x22, 0, 7, 0x21, 0, 127};

	uint8_t Thrcot_Large_Logo_Data1[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
			0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
			0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x40,
			0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xF8,
			0x1C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	uint8_t Thrcot_Large_Logo_Data2[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0C,
			0x0E, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
			0x0F, 0x0F, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F,
			0x0F, 0x0F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF, 0xC0, 0xE0, 0xF0, 0xF0,
			0xF0, 0xF0, 0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00,
			0x00, 0x80, 0xF0, 0xFC, 0x7F, 0x0F, 0xF3, 0xF0,
			0x80, 0xC0, 0xC0, 0xE0, 0xF0, 0x70, 0x30, 0x30,
			0x30, 0x10, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0,
			0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xC0,
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xF0, 0xF0, 0xC0, 0xC0, 0xFC, 0xFC, 0xFC, 0xFC,
			0xC0, 0xC0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00,
			0x80, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFE,
			0xF0, 0xF0, 0xF0, 0xF0, 0x70, 0x30, 0x10, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Large_Logo_Data3[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF, 0x03, 0x01, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0xE0,
			0xFC, 0xFF, 0x1F, 0x01, 0x00, 0x00, 0xFF, 0xFF,
			0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xF8, 0xFE, 0xFF, 0xFF, 0x07, 0x03,
			0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03,
			0x07, 0x0F, 0x00, 0x00, 0xCF, 0xCF, 0xFF, 0xFF,
			0xFF, 0xCF, 0x87, 0x03, 0x01, 0x00, 0x00, 0x01,
			0x03, 0x87, 0xCF, 0xFF, 0xFF, 0xFF, 0xCF, 0xCF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Large_Logo_Data4[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0xFC, 0xFE, 0x87, 0x03,
			0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x1F,
			0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x3E, 0x7C,
			0xF8, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x78, 0x3C,
			0x1E, 0x0F, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03,
			0x3F, 0x3F, 0x0F, 0x0F, 0xFE, 0xFC, 0xFC, 0xFE,
			0x0F, 0x0F, 0x3F, 0x3F, 0x03, 0x03, 0x03, 0x03,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x7F,
			0xE0, 0xE0, 0x70, 0x38, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Large_Logo_Data5[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07,
			0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xC0, 0xC0, 0xC0,
			0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0xF0, 0xFF, 0x7F, 0x03, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xF0, 0xF0, 0x30, 0x30, 0xF0, 0xF0, 0x30, 0x30,
			0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x80,
			0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xFC, 0xF8,
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Large_Logo_Data6[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01,
			0x03, 0x03, 0x03, 0x07, 0x07, 0x0E, 0x0E, 0x0C,
			0x1C, 0x38, 0x38, 0x30, 0x30, 0x70, 0x70, 0x60,
			0x60, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0,
			0xE0, 0xFE, 0xFF, 0xC7, 0xC0, 0x80, 0x80, 0x80,
			0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Large_Logo_Data7[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9,
			0xFF, 0x3F, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03,
			0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07,
			0x07, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E,
			0x0C, 0x0C, 0x0C, 0x1C, 0x1C, 0x1C, 0x3C, 0x3C,
			0x3C, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38,
			0x38, 0x38, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30,
			0x70, 0x70, 0x73, 0xF3, 0xF3, 0xE3, 0xE3, 0xE0,
			0xE0, 0xE0, 0xE3, 0xE3, 0xC0, 0xC0, 0xC0, 0xC0,
			0xC3, 0xC3, 0xC0, 0xC0, 0xC0, 0xC0, 0xC3, 0x83,
			0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

	uint8_t Thrcot_Large_Logo_Data8[] = {
			0x40,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
			0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Large_Logo_Range, sizeof(Large_Logo_Range), HAL_MAX_DELAY);

	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data1, sizeof(Thrcot_Large_Logo_Data1), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data2, sizeof(Thrcot_Large_Logo_Data2), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data3, sizeof(Thrcot_Large_Logo_Data3), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data4, sizeof(Thrcot_Large_Logo_Data4), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data5, sizeof(Thrcot_Large_Logo_Data5), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data6, sizeof(Thrcot_Large_Logo_Data6), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data7, sizeof(Thrcot_Large_Logo_Data7), HAL_MAX_DELAY);
	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Thrcot_Large_Logo_Data8, sizeof(Thrcot_Large_Logo_Data8), HAL_MAX_DELAY);
}

void OLED_Dot_Display(int x, int y)
{
	int Page;
	uint8_t Dot_Data;

	Page = y / 8;
	Dot_Data = 0x01 << (y % 8);

	All_Display_Data[Page][x + 1] |= Dot_Data;
}

void OLED_Horizontal_Display(int x_s, int x_e, int y)
{
	if (x_s > x_e) {
		int if_x = x_s;

		x_s = x_e;
		x_e = if_x;
	}

	int page = y / 8;
	uint8_t data = 0x01 << (y % 8);

	for (int i = x_s; i <= x_e; i++) {
		All_Display_Data[page][i+1] |= data;
	}
}

void OLED_Vartical_Display(int y_s, int y_e, int x)
{
	if (y_s > y_e) {
		int if_y = y_s;

		y_s = y_e;
		y_e = if_y;
	}

	int first_page = y_s / 8;
	int last_page = y_e / 8;

	uint8_t first_data = 0, last_data = 0;

	for (int i = first_page + 1; i <= last_page - 1; i++) {
		All_Display_Data[i][x+1] = 0xFF;
	}

	for (int i = y_s % 8; i < 8; i++) {
		first_data |= 0x01 << i;
	}

	for (int i = y_e % 8; i >= 0; i--) {
		last_data |= 0x01 << i;
	}

	All_Display_Data[first_page][x+1] |= first_data;
	All_Display_Data[last_page][x+1] |= last_data;
}

void OLED_Line_Display(int x_s, int y_s, int x_e, int y_e)
{
	int first_page, last_page;
	double Dx, Dy, angle;

	uint8_t Data_Map[64][128];

	for (int i = 0; i < 64; i++) {
		for (int j = 0; j < 128; j++) {
			Data_Map[i][j] = 0;
		}
	}

	if (((x_s > x_e) && (y_s > y_e)) || ((x_s > x_e) && (y_e > y_s))) {
		int if_x = x_s;
		int if_y = y_s;

		x_s = x_e;
		x_e = if_x;
		y_s = y_e;
		y_e = if_y;
	}

	Dx = x_e - x_s;
	Dy = y_e - y_s;

	if (Dx == 0.0) {
		OLED_Vartical_Display(y_s, y_e, x_s);

		return;
	} else if (Dy == 0.0) {
		OLED_Horizontal_Display(x_s, x_e, y_s);

		return;
	} else {
		angle = atan2(Dy, Dx) * 180.0 / PI;

		if ((angle > 45.0) || (angle < -45.0)) {
			if (Dy < 0.0) {
				for (int i = y_e; i <= y_s; i++) {
					int x_map = (Dx / Dy) * (i - y_s) + x_s;
					int y_map = i;

					Data_Map[y_map][x_map] = 1;
				}
			} else {
				for (int i = y_s; i <= y_e; i++) {
					int x_map = (Dx / Dy) * (i - y_s) + x_s;
					int y_map = i;

					Data_Map[y_map][x_map] = 1;
				}
			}
		} else {
			for (int i = x_s; i <= x_e; i++) {
				int x_map = i;
				int y_map = (Dy / Dx) * (i - x_s) + y_s;

				Data_Map[y_map][x_map] = 1;
			}
		}

		if (y_s > y_e) {
			first_page = y_e / 8;
			last_page = y_s / 8;
		} else {
			first_page = y_s / 8;
			last_page = y_e / 8;
		}

		for (int i = first_page; i <= last_page; i++) {
			for (int j = x_s; j <= x_e; j++) {
				int x_map = j;
				int y_map = i * 8 + 7;

				All_Display_Data[i][j+1] |= (Data_Map[y_map][x_map] << 7)
											+ (Data_Map[y_map-1][x_map] << 6)
											+ (Data_Map[y_map-2][x_map] << 5)
											+ (Data_Map[y_map-3][x_map] << 4)
											+ (Data_Map[y_map-4][x_map] << 3)
											+ (Data_Map[y_map-5][x_map] << 2)
											+ (Data_Map[y_map-6][x_map] << 1)
											+ Data_Map[y_map-7][x_map];
			}
		}
	}
}

void OLED_Circle_Draw(int x_0, int y_0, int radiation)
{
	int x = radiation;
	int y = 0;
	int F = -2 * radiation + 3;

	while (x >= y) {
		OLED_Dot_Display(x_0 + x, y_0 + y);
		OLED_Dot_Display(x_0 + x, y_0 - y);
		OLED_Dot_Display(x_0 - x, y_0 + y);
		OLED_Dot_Display(x_0 - x, y_0 - y);
		OLED_Dot_Display(x_0 + y, y_0 + x);
		OLED_Dot_Display(x_0 + y, y_0 - x);
		OLED_Dot_Display(x_0 - y, y_0 + x);
		OLED_Dot_Display(x_0 - y, y_0 - x);

		if (F >= 0) {
			x--;
			F -= 4 * x;
		}

		y++;
		F += 4 * y +2;
	}
}

void OLED_DataClear(void)
{
	for (int i = 0; i < 8; i++) {
		All_Display_Data[i][0] = 0x40;

		for (int j = 1; j <= 128; j++) {
			All_Display_Data[i][j] = 0x00;
		}
	}
}

void OLED_AllClear(I2C_HandleTypeDef *hi2c)
{
	uint8_t Clear_Range[] = {0x00, 0x22, 0, 7, 0x21, 0, 127};

	for (int i = 0; i < 8; i++) {
		All_Display_Data[i][0] = 0x40;

		for (int j = 1; j <= 128; j++) {
			All_Display_Data[i][j] = 0x00;
		}
	}

	HAL_I2C_Master_Transmit(hi2c, OLED_ADR, Clear_Range, sizeof(Clear_Range), HAL_MAX_DELAY);

	for (int i = 0; i < 8; i++) {
		HAL_I2C_Master_Transmit(hi2c, OLED_ADR, All_Display_Data[i], sizeof(All_Display_Data[i]), HAL_MAX_DELAY);
	}
}

void OLED_Char_Print(uint8_t *message, int x, int y)
{
	int MessageSize = 0;
	int Flag = 0;
	int X_position = 0;
	int Y_position = 0;
	int Y_page = 0;

	uint8_t MSB_Data = 0;
	uint8_t LSB_Data = 0;
	uint8_t Message_Data[6];

	while (message[MessageSize] != 0x00) {
		switch (message[MessageSize]) {
			case 0x21:
				Font_Exclamation(Message_Data);
				break;

			case 0x22:
				Font_Double_Quotation(Message_Data);
				break;

			case 0x23:
				Font_Sharp(Message_Data);
				break;

			case 0x24:
				Font_Dollar(Message_Data);
				break;

			case 0x25:
				Font_Percent(Message_Data);
				break;

			case 0x26:
				Font_And(Message_Data);
				break;

			case 0x27:
				Font_Single_Quotation(Message_Data);
				break;

			case 0x28:
				Font_Left_Bracket(Message_Data);
				break;

			case 0x29:
				Font_Right_Bracket(Message_Data);
				break;

			default:
				Flag = 1;
				break;
		}

		if (Flag != 1) {
			X_position = x + MessageSize * 6;
			Y_position = y + 7;
			Y_page = Y_position / 8;

			if (Y_position % 8 == 7) {
				for (int i = 1; i <= 6; i++) {
					All_Display_Data[Y_page][X_position + i] |= Message_Data[i-1];
				}
			} else {
				uint8_t Y_page2 = Y_page - 1;

				for (int i = 1; i <= 6; i++) {
					MSB_Data = Message_Data[i-1] >> (Y_position % 8);

					for (int j = 0; j < (Y_position % 8); j++) {
						LSB_Data |= 0x80 >> j;
					}

					LSB_Data = Message_Data[i-1] & LSB_Data;

					All_Display_Data[Y_page2][X_position + i] |= LSB_Data;
					All_Display_Data[Y_page][X_position + i] |= MSB_Data;
				}
			}
		}

		MessageSize++;
	}
}
