#include <stm32f446_sys.h>
#include <stm32f446_gpio.h>
#include <stm32f446_usart.h>

#define LD2			PA5
#define B1			PC13

#define LD2_ON		(GPIOA->ODR |= 1UL << 5)
#define LD2_OFF		(GPIOA->ODR &= ~(1UL << 5))
#define LD2_TOGGLE	(GPIOA->ODR ^= (1UL << 5))

#define B1_STATES	((GPIOC->IDR & (1UL << 13)) >> 13)

/*USART2 callback function*/
void USART2_Transmit_CmpltCallback(void);
void USART2_Receive_OvrunCallback(void);
void USART2_Receive_CmpltCallback(void);

bool usart2_tx_state = false;
bool usart2_rx_state = false;

int main(void)
{
	RCC_Init();
	SysTick_Init();

	pinMode(LD2, OUTPUT);
	pinMode(B1, INPUT);

	Serial.init(115200);
	Serial.setCallback(TX_COMPLETE, USART2_Transmit_CmpltCallback);
	Serial.setCallback(RX_OVERRUN, USART2_Receive_OvrunCallback);
	Serial.setCallback(RX_COMPLETE, USART2_Receive_CmpltCallback);

	while(1)
	{
		Serial.println(-30000);
		delay(10000);
	}

	return 0;
}

void USART2_Transmit_CmpltCallback(void)
{
	usart2_tx_state = true;
}

void USART2_Receive_OvrunCallback(void)
{
	usart2_rx_state = true;
}

void USART2_Receive_CmpltCallback(void)
{
	usart2_rx_state = true;
}
