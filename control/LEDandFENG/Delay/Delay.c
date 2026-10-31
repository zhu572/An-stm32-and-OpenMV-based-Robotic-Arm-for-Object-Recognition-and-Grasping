#include "Delay.h"

#if defined(USE_HAL_DRIVER)
#include "stm32f1xx.h"        /* HAL 工程 */
#else
#include "stm32f10x.h"        /* 标准外设库 / 寄存器工程 */
#endif

/* SysTick 一次最多计 0xFFFFFF 个内核时钟：72MHz 下约 233ms */
#define DELAY_TICKS_MAX   0x00FFFFFFUL
/* 每次交给 SysTick 的最大微秒数：100ms，160MHz 以内都不会溢出 */
#define DELAY_US_CHUNK    100000UL

static uint32_t s_fac_us = 0;   /* 1us 对应多少个内核时钟 */

void Delay_Init(void)
{
#if defined(USE_HAL_DRIVER) || defined(USE_STDPERIPH_DRIVER)
    SystemCoreClockUpdate();                 /* 按实际 PLL 配置刷新主频 */
#endif
    s_fac_us = SystemCoreClock / 1000000UL;
    if (s_fac_us == 0UL)
    {
        s_fac_us = 1UL;
    }
}

/* 轮询等待 ticks 个内核时钟。
   进出时保存并还原 SysTick 的 CTRL/LOAD/VAL，
   所以工程里 HAL/SPL 已把 SysTick 当 1ms 时基用也不会被破坏 */
static void Delay_WaitTicks(uint32_t ticks)
{
    uint32_t ctrl = SysTick->CTRL;
    uint32_t load = SysTick->LOAD;
    uint32_t val  = SysTick->VAL;

    SysTick->CTRL = 0UL;                                   /* 先停，改配置期间不乱动 */
    SysTick->LOAD = ticks - 1UL;                           /* 计数 ticks 次到 0 */
    SysTick->VAL  = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;  /* 内核时钟 + 关中断 + 启动计数 + 仅轮询 COUNTFLAG */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
    {
    }
    SysTick->CTRL = 0UL;

    SysTick->LOAD = load;                                  /* 还原现场 */
    SysTick->VAL  = val;
    SysTick->CTRL = ctrl;
}

void Delay_us(uint32_t us)
{
    if (us == 0UL)
    {
        return;
    }
    if (s_fac_us == 0UL)
    {
        Delay_Init();
    }

    while (us > 0UL)
    {
        uint32_t chunk = (us > DELAY_US_CHUNK) ? DELAY_US_CHUNK : us;
        uint32_t ticks = chunk * s_fac_us;

        if (ticks > DELAY_TICKS_MAX)
        {
            ticks = DELAY_TICKS_MAX;
        }
        Delay_WaitTicks(ticks);
        us -= chunk;
    }
}

/* 逐毫秒累加：不会溢出的同时，长延时的累计误差也只有每毫秒一次配置开销 */
void Delay_ms(uint32_t ms)
{
    while (ms-- > 0UL)
    {
        Delay_us(1000UL);
    }
}

void Delay_s(uint32_t s)
{
    while (s-- > 0UL)
    {
        Delay_ms(1000UL);
    }
}
