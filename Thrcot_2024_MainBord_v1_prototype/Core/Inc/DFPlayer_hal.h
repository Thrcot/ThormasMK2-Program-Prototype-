/*
 * DFPlayer_hal.h
 *
 *  Created on: Aug 2, 2024
 *      Author: neoki
 */

#ifndef INC_DFPLAYER_HAL_H_
#define INC_DFPLAYER_HAL_H_

#include "stm32f4xx_hal.h"

#define DFP_OK 1
#define DFP_NG 0
#define DFP_ENABLE 1
#define DFP_DISABLE 0

#define PARA 0x00           ///NEXT,PAUSE,STOP,PLAY,RANDのLSB
#define FD_NEXT 0x01        ///次の曲を再生
#define FD_PREVIOUS 0x02    ///前の曲を再生
#define FD_SPEC 0x03        ///特定のトラック(曲)を再生
#define FD_VOL_INC 0x04     ///音量+
#define FD_VOL_DEC 0x05     ///音量-
#define FD_VOL 0x06         ///音量の設定
#define FD_SPEC_REP 0x08    ///特定の曲、ファイル、ランダムリピート
#define FD_LOW 0x0A         ///パワーオフモード
#define FD_NW 0x0B          ///通常起動
#define FD_RES 0x0C         ///モジュールをリセット
#define FD_PLAY 0x0D        ///曲を再生
#define FD_PAUSE 0x0E       ///曲の一時停止
#define FD_FOLDER_PLAY 0x0F ///フォルダの再生
#define FD_VOL_AD 0x10      ///ボリュームの調整
#define FD_REPEAT 0x11      ///曲のリピート再生

extern UART_HandleTypeDef *DFP_serial;

int DFP_Send_Data(uint8_t cmd, uint8_t MSB, uint8_t LSB, uint8_t feedback);

void DFP_Init(UART_HandleTypeDef *serial);
void DFP_Restart(void);
void DFP_Pause(void);
void DFP_Volume(int volume);
void DFP_Reset(void);
void DFP_Play(int fileNUM);

#endif /* INC_DFPLAYER_HAL_H_ */
