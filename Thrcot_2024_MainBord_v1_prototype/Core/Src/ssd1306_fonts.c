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

void Font_Asterisk(uint8_t *Font_Data)
{
	Font_Data[0] = 0x2A;
	Font_Data[1] = 0x1C;
	Font_Data[2] = 0x7F;
	Font_Data[3] = 0x1C;
	Font_Data[4] = 0x2A;
	Font_Data[5] = 0x00;
}

void Font_Plus(uint8_t *Font_Data)
{
	Font_Data[0] = 0x08;
	Font_Data[1] = 0x08;
	Font_Data[2] = 0x3E;
	Font_Data[3] = 0x08;
	Font_Data[4] = 0x08;
	Font_Data[5] = 0x00;
}
void Font_Comma(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x80;
	Font_Data[2] = 0x70;
	Font_Data[3] = 0x30;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_Minus(uint8_t *Font_Data)
{
	Font_Data[0] = 0x08;
	Font_Data[1] = 0x08;
	Font_Data[2] = 0x08;
	Font_Data[3] = 0x08;
	Font_Data[4] = 0x08;
	Font_Data[5] = 0x00;
}

void Font_Period(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x00;
	Font_Data[2] = 0x60;
	Font_Data[3] = 0x60;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_Slash(uint8_t *Font_Data)
{
	Font_Data[0] = 0x20;
	Font_Data[1] = 0x10;
	Font_Data[2] = 0x08;
	Font_Data[3] = 0x04;
	Font_Data[4] = 0x02;
	Font_Data[5] = 0x00;
}

void Font_0(uint8_t *Font_Data)
{
	Font_Data[0] = 0x3E;
	Font_Data[1] = 0x51;
	Font_Data[2] = 0x49;
	Font_Data[3] = 0x45;
	Font_Data[4] = 0x3E;
	Font_Data[5] = 0x00;
}

void Font_1(uint8_t *Font_Data)
{
	Font_Data[0] = 0x00;
	Font_Data[1] = 0x42;
	Font_Data[2] = 0x7F;
	Font_Data[3] = 0x40;
	Font_Data[4] = 0x00;
	Font_Data[5] = 0x00;
}

void Font_2(uint8_t *Font_Data)
{
	Font_Data[0] = 0x72;
	Font_Data[1] = 0x49;
	Font_Data[2] = 0x49;
	Font_Data[3] = 0x49;
	Font_Data[4] = 0x46;
	Font_Data[5] = 0x00;
}

void Font_3(uint8_t *Font_Data)
{
	Font_Data[0] = 0x21;
	Font_Data[1] = 0x41;
	Font_Data[2] = 0x49;
	Font_Data[3] = 0x4D;
	Font_Data[4] = 0x33;
	Font_Data[5] = 0x00;
}

void Font_4(uint8_t *Font_Data)
{
	Font_Data[0] = 0x18;
	Font_Data[1] = 0x14;
	Font_Data[2] = 0x12;
	Font_Data[3] = 0x7F;
	Font_Data[4] = 0x10;
	Font_Data[5] = 0x00;
}

void Font_5(uint8_t *Font_Data)
{
	Font_Data[0] = 0x27;
	Font_Data[1] = 0x45;
	Font_Data[2] = 0x45;
	Font_Data[3] = 0x45;
	Font_Data[4] = 0x39;
	Font_Data[5] = 0x00;
}

void Font_6(uint8_t *Font_Data)
{
	Font_Data[0] = 0x3C;
	Font_Data[1] = 0x4A;
	Font_Data[2] = 0x49;
	Font_Data[3] = 0x49;
	Font_Data[4] = 0x31;
	Font_Data[5] = 0x00;
}

void Font_7(uint8_t *Font_Data)
{
	Font_Data[0] = 0x41;
	Font_Data[1] = 0x21;
	Font_Data[2] = 0x11;
	Font_Data[3] = 0x09;
	Font_Data[4] = 0x07;
	Font_Data[5] = 0x00;
}

void Font_8(uint8_t *Font_Data)
{
	Font_Data[0] = 0x36;
	Font_Data[1] = 0x49;
	Font_Data[2] = 0x49;
	Font_Data[3] = 0x49;
	Font_Data[4] = 0x36;
	Font_Data[5] = 0x00;
}

void Font_9(uint8_t *Font_Data)
{
	Font_Data[0] = 0x46;
	Font_Data[1] = 0x49;
	Font_Data[2] = 0x49;
	Font_Data[3] = 0x29;
	Font_Data[4] = 0x1E;
	Font_Data[5] = 0x00;
}

void Font_Large_A(uint8_t *Font_Data)
{

}
