#include "stm32f10x.h"                  // Device header
#include "LED_DRIVER.h"

/* 按端口打开对应的 APB2 时钟：C 里不能把变量拼进宏名字，
   所以要一个一个判断 */
static void LED_ClockEnable(GPIO_TypeDef *GPIOx)
{
	if (GPIOx == GPIOA)      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (GPIOx == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if (GPIOx == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if (GPIOx == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	else if (GPIOx == GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

/* 初始化某个端口的某个引脚为推挽输出 */
void InitGPIO(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	LED_ClockEnable(GPIOx);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	GPIO_SetBits(GPIOx, pin);          /* 默认输出高电平 = 灯灭（低电平点亮） */
}

void LED_ON(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	GPIO_ResetBits(GPIOx, pin);
}

void LED_OFF(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	GPIO_SetBits(GPIOx, pin);
}
