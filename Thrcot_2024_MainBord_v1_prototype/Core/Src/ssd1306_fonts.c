/*
 * ssd1306_fonts.c
 *
 *  Created on: Aug 3, 2024
 *      Author: neoki
 */

#include "ssd1306_fonts.h"

void Font_Exclamation(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x00;
	Font_Data[2] = 0x5F;
	Font_Data[3] = 0x00;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

uint8_t *Font_A(void)
{
	return 0;
}
