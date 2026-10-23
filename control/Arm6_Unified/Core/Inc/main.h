#ifndef MAIN_H
#define MAIN_H
#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim2, htim3;
extern UART_HandleTypeDef huart1, huart3;
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#endif
