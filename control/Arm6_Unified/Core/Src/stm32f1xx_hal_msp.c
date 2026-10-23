#include "main.h"

void HAL_MspInit(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* Keep PA13/PA14 SWD available. */
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_AFIO_REMAP_TIM2_DISABLE();
    } else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE(); __HAL_AFIO_REMAP_TIM3_DISABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    if (htim->Instance == TIM2) gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    else if (htim->Instance == TIM3) gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    else return;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART1) {
        /* 电脑链路：PA9 TX / PA10 RX */
        __HAL_RCC_USART1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_AFIO_REMAP_USART1_DISABLE();
        gpio.Pin = GPIO_PIN_9; gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Pin = GPIO_PIN_10; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &gpio);
        HAL_NVIC_SetPriority(USART1_IRQn, 2u, 0u);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (huart->Instance == USART3) {
        /* OpenMV 链路：PB10 TX / PB11 RX（USART2 的 PA2/PA3 已被 S3/S4 占用） */
        __HAL_RCC_USART3_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_10; gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH; gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &gpio);
        gpio.Pin = GPIO_PIN_11; gpio.Mode = GPIO_MODE_INPUT; gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_NVIC_SetPriority(USART3_IRQn, 3u, 0u);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}
