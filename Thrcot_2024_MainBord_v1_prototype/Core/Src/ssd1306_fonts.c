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

void Font_Double_Quotation(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x07;
	Font_Data[2] = 0x00;
	Font_Data[3] = 0x07;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_Sharp(uint8_t *Font_Data)
{
	Font_Data[0] = 0x14;
	Font_Data[1] = 0x7F;
	Font_Data[2] = 0x14;
	Font_Data[3] = 0x7F;
	Font_Data[4] = 0x14;
	Font_Data[5] = 0x00;
}

void Font_Dollar(uint8_t *Font_Data)
{
	Font_Data[0] = 0x24;
	Font_Data[1] = 0x2A;
	Font_Data[2] = 0x7F;
	Font_Data[3] = 0x2A;
	Font_Data[4] = 0x12;
	Font_Data[5] = 0x00;
}

void Font_Percent(uint8_t *Font_Data)
{
	Font_Data[0] = 0x23;
	Font_Data[1] = 0x13;
	Font_Data[2] = 0x08;
	Font_Data[3] = 0x64;
	Font_Data[4] = 0x62;
	Font_Data[5] = 0x00;
}

void Font_And(uint8_t *Font_Data)
{
	Font_Data[0] = 0x36;
	Font_Data[1] = 0x49;
	Font_Data[2] = 0x56;
	Font_Data[3] = 0x20;
	Font_Data[4] = 0x50;
	Font_Data[5] = 0x00;
}

void Font_Single_Quotation(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x08;
	Font_Data[2] = 0x07;
	Font_Data[3] = 0x03;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_Left_Bracket(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x1C;
	Font_Data[2] = 0x22;
	Font_Data[3] = 0x41;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_Right_Bracket(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x41;
	Font_Data[2] = 0x22;
	Font_Data[3] = 0x1C;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_Large_A(uint8_t *Font_Data)
{

}
