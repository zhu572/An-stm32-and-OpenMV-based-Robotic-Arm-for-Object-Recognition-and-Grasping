#ifndef __LED_DRIVER_H
#define __LED_DRIVER_H

#include "stm32f10x.h"                  // Device header

/* GPIOx: GPIOA/GPIOB/... ，pin: GPIO_Pin_0 ~ GPIO_Pin_15 */
void InitGPIO(GPIO_TypeDef *GPIOx, uint16_t pin);
void LED_ON(GPIO_TypeDef *GPIOx, uint16_t pin);
void LED_OFF(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif
