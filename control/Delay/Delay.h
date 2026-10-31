#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>

/* =========================================================
 * Delay.h -- STM32F103C8T6 精确延时（SysTick 轮询，72MHz 下 1us 分辨率）
 *   不占用中断、不占用定时器、不依赖 HAL/SPL 的时基
 *   用法：
 *       #include "Delay.h"
 *       Delay_Init();        // 可选，不调用则首次延时自动初始化
 *       Delay_us(20);
 *       Delay_ms(500);
 *       Delay_s(1);
 * ========================================================= */

void Delay_Init(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s (uint32_t s );

#endif /* __DELAY_H__ */
