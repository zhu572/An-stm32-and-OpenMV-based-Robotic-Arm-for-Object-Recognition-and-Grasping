/* ============================================================================
 * main.c -- Arm6_Test_ServoAll   六路舵机固定角度测试
 *
 * 原 Arm6_Test_Servo1 的扩展版：不再只动 S1，而是把 TIM2/TIM3 的六个通道
 * 全部输出同一个固定角度，用来一次性验证「六个舵机的接线、供电、方向」。
 *
 *   S1..S4 = PA0..PA3  = TIM2_CH1..CH4
 *   S5, S6 = PA6, PA7  = TIM3_CH1, CH2
 *   全部输出 FIXED_ANGLE_X10 对应的脉宽（默认 90.0° → 1500 µs）
 *
 * 不依赖 HAL，直接操作寄存器；编译进来只有本文件 + 启动文件 + system 文件。
 *
 * 注意：本工程上电立即输出 PWM（正式固件要 $ON#）。上电前请确认机械臂
 *       不会因为同时跳到中位而互相碰撞。
 * ========================================================================== */
#include "stm32f1xx.h"

/* ---------------------------------------------------------------- 可调参数 -- */
#define FIXED_ANGLE_X10     900u  /* 固定角度，单位 0.1°：900 = 90.0°        */

/* 1 = 六个通道恒定输出 FIXED_ANGLE_X10，完全不理串口（当前行为）
 * 0 = 每收到一帧视觉数据，六路一起在 TOGGLE_LOW / TOGGLE_HIGH 之间翻一次 */
#define SERVO_STATIC_ONLY   1

#define TOGGLE_LOW_X10      900u  /* 仅 SERVO_STATIC_ONLY=0 时使用：90.0°     */
#define TOGGLE_HIGH_X10     990u  /*                                99.0°     */

/* ------------------------------------------------------------ 时钟 64 MHz -- */
static void clock_64mhz_hsi(void)
{
    /* HSI 8 MHz / 2 × 16 = 64 MHz */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) { }

    RCC->CFGR &= ~RCC_CFGR_PLLSRC;          /* PLL 源 = HSI/2 */
    RCC->CFGR &= ~RCC_CFGR_PLLMULL;
    RCC->CFGR |= RCC_CFGR_PLLMULL16;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u) { }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    /* HCLK=64 MHz，PCLK1=32 MHz（TIM2/TIM3 时钟仍为 64 MHz），PCLK2=64 MHz */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
}

/* ------------------------------------------------- 角度(0.1°) → 脉宽(µs) -- */
/* 线性映射：0.0° → 1000 µs，180.0° → 2000 µs（与正式固件同一套假设） */
static uint16_t angle_to_us(uint32_t tenths)
{
    return (uint16_t)(1000u + (tenths * 1000u + 900u) / 1800u);
}

/* ------------------------------------------------------------------- GPIO -- */
static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    /* PA0..PA3（TIM2_CH1..4）与 PA6、PA7（TIM3_CH1..2）：复用推挽 50 MHz
     * CRL 每个引脚 4 bit：0b1011 = 0xB
     *   bit0..15  = PA0..PA3
     *   bit24..31 = PA6、PA7
     * 中间 PA4/PA5（bit16..23）保持复位默认值 */
    GPIOA->CRL = (GPIOA->CRL & 0x00FF0000u) | 0xBB00BBBBu;

    /* PB10 = USART3_TX 复用推挽 50 MHz，PB11 = USART3_RX 浮空输入 */
    GPIOB->CRH = (GPIOB->CRH & ~0x0000FF00u) | 0x00004B00u;
}

/* --------------------------------------------- TIM2 CH1..CH4（S1..S4） -- */
static void tim2_pwm_init(uint16_t pulse_us)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC  = 63u;           /* 64 MHz / 64 = 1 MHz → 1 计数 = 1 µs */
    TIM2->ARR  = 19999u;        /* 20 ms 周期 → 50 Hz                */
    TIM2->CCR1 = pulse_us;
    TIM2->CCR2 = pulse_us;
    TIM2->CCR3 = pulse_us;
    TIM2->CCR4 = pulse_us;

    /* CH1/CH2 在 CCMR1，CH3/CH4 在 CCMR2；OCxM = 110 即 PWM mode 1 */
    TIM2->CCMR1 = (uint16_t)((TIM2->CCMR1 & (uint16_t)~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M))
                             | (0x6u << 4) | (0x6u << 12));
    TIM2->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);

    TIM2->CCMR2 = (uint16_t)((TIM2->CCMR2 & (uint16_t)~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M))
                             | (0x6u << 4) | (0x6u << 12));
    TIM2->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);

    TIM2->CR1  |= TIM_CR1_ARPE;
    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    TIM2->CR1  |= TIM_CR1_CEN;
}

/* ---------------------------------------------- TIM3 CH1..CH2（S5、S6） -- */
static void tim3_pwm_init(uint16_t pulse_us)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC  = 63u;
    TIM3->ARR  = 19999u;
    TIM3->CCR1 = pulse_us;
    TIM3->CCR2 = pulse_us;

    TIM3->CCMR1 = (uint16_t)((TIM3->CCMR1 & (uint16_t)~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M))
                             | (0x6u << 4) | (0x6u << 12));
    TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);

    TIM3->CR1  |= TIM_CR1_ARPE;
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM3->CR1  |= TIM_CR1_CEN;
}

/* ------------------------------------------------------- USART3 115200 8N1 -- */
static void usart3_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* PCLK1 = 32 MHz：BRR = 32e6 / 115200 ≈ 277.8 → 278 */
    USART3->BRR = (uint16_t)((32000000u + (115200u / 2u)) / 115200u);

    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

#if !SERVO_STATIC_ONLY
/* 六路一起改脉宽（仅触发模式需要） */
static void set_all_channels(uint16_t pulse_us)
{
    TIM2->CCR1 = pulse_us;
    TIM2->CCR2 = pulse_us;
    TIM2->CCR3 = pulse_us;
    TIM2->CCR4 = pulse_us;
    TIM3->CCR1 = pulse_us;
    TIM3->CCR2 = pulse_us;
}
#endif

/* ------------------------------------------------------------------- main -- */
int main(void)
{
    uint16_t pulse = angle_to_us(FIXED_ANGLE_X10);

    clock_64mhz_hsi();
    gpio_init();
    tim2_pwm_init(pulse);       /* S1..S4 */
    tim3_pwm_init(pulse);       /* S5、S6 */
    usart3_init();

#if SERVO_STATIC_ONLY
    /* 静态保持：脉宽已经写进 CCR，定时器硬件自己一直输出 50 Hz，
     * CPU 不需要再做任何事。 */
    for (;;) {
        __NOP();
    }
#else
    {
        uint8_t state = 0u;      /* 帧状态机 */
        uint8_t count = 0u;

        for (;;) {
            if ((USART3->SR & USART_SR_RXNE) == 0u) continue;

            {
                uint8_t byte = (uint8_t)(USART3->DR & 0xFFu);
                int trigger = 0;

                switch (state) {
                case 0u:
                    if (byte == 0x2Cu) state = 1u;
                    break;
                case 1u:
                    if (byte == 0x12u) { state = 2u; count = 0u; }
                    else if (byte != 0x2Cu) state = 0u;
                    break;
                default:
                    if (byte == 0x5Bu) { state = 0u; trigger = 1; }
                    else if (++count > 5u) state = 0u;
                    break;
                }

                if (trigger != 0) {
                    pulse = (pulse == angle_to_us(TOGGLE_LOW_X10))
                              ? angle_to_us(TOGGLE_HIGH_X10)
                              : angle_to_us(TOGGLE_LOW_X10);
                    set_all_channels(pulse);
                }
            }
        }
    }
#endif
}
