/*
 * DFPlayer_hal.c
 *
 *  Created on: Aug 2, 2024
 *      Author: neoki
 */

#include "DFPlayer_hal.h"

UART_HandleTypeDef *DFP_serial;

int DFP_Send_Data(uint8_t cmd, uint8_t MSB, uint8_t LSB, uint8_t feedback)
{
	uint8_t DataSend[] = {0x7E, 0xFF, 0x06, 0x00, 0x01, 0x00, 0x00, 0xEF};
	uint8_t FeedbackData[] = {0x7E, 0xFF, 0x06, 0x41, 0x00, 0x00, 0x00, 0xFE, 0xBA, 0xEF};
	uint8_t ReceiveData[20];

	int ReturnData = 0;

	DataSend[3] = cmd;
	DataSend[5] = MSB;
	DataSend[6] = LSB;

	HAL_UART_Transmit(DFP_serial, DataSend, sizeof(DataSend), 1000);

	if (feedback == DFP_ENABLE) {
		HAL_UART_Receive(DFP_serial, ReceiveData, 10, 1000);

		int count = 0;
		for (count = 0; count < 10; count++) {
			if (ReceiveData[count] != FeedbackData[count]) {
				break;
			}
		}

		ReturnData = (count == 10) ? DFP_OK : DFP_NG;
	}

	return ReturnData;
}

void DFP_Init(UART_HandleTypeDef *serial)
{
	DFP_serial = serial;

	while (DFP_Send_Data(0x0B, 0x00, 0x00, DFP_ENABLE) != DFP_OK) {
		;
	}
}
