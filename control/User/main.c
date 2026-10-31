#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Serial.h"
#include "BusServo_Driver.h"
#include "Delay.h"


static void Show_Frame(uint8_t color, uint8_t cx, uint8_t cy, uint16_t n)
{
	char c = '?';

	if (color >= 'A' && color <= 'Z')
	{
		c = (char)color;
	}

	OLED_ShowString(1, 1, "COLOR: ");
	OLED_ShowChar(1, 8, c);

	OLED_ShowString(2, 1, "X: ");
	OLED_ShowNum(2, 4, cx, 3);

	OLED_ShowString(3, 1, "Y: ");
	OLED_ShowNum(3, 4, cy, 3);

	OLED_ShowString(4, 1, "N: ");
	OLED_ShowNum(4, 4, n, 5);
}

int main(void)
{
	OLED_Init();
	Delay_Init();
	Serial_Init();
	OLED_Clear();
	BusServo_Init();

	Show_Frame('A', 0, 0, 0);
	placeintial();

	while (1)
	{
	}
}
