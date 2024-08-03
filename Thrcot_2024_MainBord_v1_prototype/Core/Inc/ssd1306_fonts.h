/*
 * ssd1306_fonts.h
 *
 *  Created on: Aug 3, 2024
 *      Author: neoki
 */

#ifndef INC_SSD1306_FONTS_H_
#define INC_SSD1306_FONTS_H_

#include "stm32f4xx_hal.h"

void Font_Exclamation(uint8_t *Font_Data);
void Font_Double_Quotation(uint8_t *Font_Data);
void Font_Sharp(uint8_t *Font_Data);
void Font_Dollar(uint8_t *Font_Data);
void Font_Percent(uint8_t *Font_Data);
void Font_And(uint8_t *Font_Data);
void Font_Single_Quotation(uint8_t *Font_Data);
void Font_Left_Bracket(uint8_t *Font_Data);
void Font_Right_Bracket(uint8_t *Font_Data);

void Font_Large_A(uint8_t *Font_Data);

#endif /* INC_SSD1306_FONTS_H_ */
