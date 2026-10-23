#include "app_hal.h"
#include "main.h"
#include "arm.h"
#include "pipeline.h"
#include "vision_frame.h"
#include "arm_config.h"
#include <string.h>

/* Kept here as well as main.h so CubeMX regeneration of main.h is harmless. */
extern TIM_HandleTypeDef htim2, htim3;
extern UART_HandleTypeDef huart1, huart3;

/* ---------------------------------------------------------------- */
/* 两路串口各自的接收环形缓冲                                        */
/*   USART1 (PA9/PA10)  : 电脑 → ASCII 命令，256 B                   */
/*   USART3 (PB10/PB11) : OpenMV → 二进制视觉帧，64 B                */
/* 中断里只搬字节，解析全部在主循环。                                */
/* ---------------------------------------------------------------- */
#define RX1_SIZE 256u
#define RX3_SIZE 64u

typedef struct {
    volatile uint8_t *ring;
    uint16_t mask;
    volatile uint16_t *head;
    volatile uint16_t *tail;
    volatile uint8_t *fault;
} rx_ring_t;

static uint8_t rx1_byte, rx3_byte;
static volatile uint8_t rx1_buf[RX1_SIZE], rx3_buf[RX3_SIZE];
static volatile uint16_t rx1_head, rx1_tail, rx3_head, rx3_tail;
static volatile uint8_t rx1_fault, rx3_fault;
static volatile uint8_t rx1_rearm, rx3_rearm;

static rx_ring_t ring1 = {(volatile uint8_t *)rx1_buf, RX1_SIZE - 1u,
                          (volatile uint16_t *)&rx1_head, (volatile uint16_t *)&rx1_tail,
                          (volatile uint8_t *)&rx1_fault};
static rx_ring_t ring3 = {(volatile uint8_t *)rx3_buf, RX3_SIZE - 1u,
                          (volatile uint16_t *)&rx3_head, (volatile uint16_t *)&rx3_tail,
                          (volatile uint8_t *)&rx3_fault};

static arm_t arm;
static vision_parser_t vision;
static bool pwm_running;

static const uint32_t channels[ARM_SERVO_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_1, TIM_CHANNEL_2
};
static TIM_HandleTypeDef *timer(unsigned k) { return k < 4u ? &htim2 : &htim3; }

void arm_port_pwm(unsigned channel, uint16_t pulse_us) {
    if (channel < ARM_SERVO_COUNT) __HAL_TIM_SET_COMPARE(timer(channel), channels[channel], pulse_us);
}

void arm_port_enable(bool enable) {
    if (enable == pwm_running) return;
    if (enable) {
        /* Transfer preload registers before enabling pins. */
        (void)HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
        (void)HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    }
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) {
        HAL_StatusTypeDef s = enable ? HAL_TIM_PWM_Start(timer(k), channels[k]) : HAL_TIM_PWM_Stop(timer(k), channels[k]);
        if (s != HAL_OK) Error_Handler();
    }
    pwm_running = enable;
}

void arm_port_reply(const char *text) {
    /* Main-loop only. Never print from receive interrupts. */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50u);
}

/* ---------------------------------------------------------------- */
/* 接收中断：只入队 + 重新武装                                        */
/* ---------------------------------------------------------------- */
static void rearm(UART_HandleTypeDef *h, uint8_t *byte, volatile uint8_t *flag) {
    if (HAL_UART_Receive_IT(h, byte, 1u) != HAL_OK) *flag = 1u;
}

static void push_byte(volatile uint8_t *buf, uint16_t size,
                      volatile uint16_t *head, volatile uint16_t *tail,
                      volatile uint8_t *fault, uint8_t byte) {
    uint16_t next = (uint16_t)((*head + 1u) & (size - 1u));
    if (next == *tail) *fault = 1u;
    else { buf[*head] = byte; *head = next; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        push_byte(rx1_buf, RX1_SIZE, &rx1_head, &rx1_tail, &rx1_fault, rx1_byte);
        rearm(&huart1, &rx1_byte, &rx1_rearm);
    } else if (huart->Instance == USART3) {
        push_byte(rx3_buf, RX3_SIZE, &rx3_head, &rx3_tail, &rx3_fault, rx3_byte);
        rearm(&huart3, &rx3_byte, &rx3_rearm);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        rx1_fault = 1u;
        (void)HAL_UART_AbortReceive(&huart1);
        rearm(&huart1, &rx1_byte, &rx1_rearm);
    } else if (huart->Instance == USART3) {
        rx3_fault = 1u;
        (void)HAL_UART_AbortReceive(&huart3);
        rearm(&huart3, &rx3_byte, &rx3_rearm);
    }
}

/* 取一个字节：-1 空，-2 溢出/错误（已清空缓冲）。 */
static int ring_take(rx_ring_t *r) {
    int result = -1;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (*r->fault) {
        *r->fault = 0u;
        *r->tail = *r->head;
        result = -2;
    } else if (*r->tail != *r->head) {
        result = r->ring[*r->tail];
        *r->tail = (uint16_t)((*r->tail + 1u) & r->mask);
    }
    __set_PRIMASK(mask);
    return result;
}

void App_Init(void) {
    rx1_head = rx1_tail = 0u;
    rx3_head = rx3_tail = 0u;
    rx1_fault = rx3_fault = 0u;
    rx1_rearm = rx3_rearm = 0u;
    pwm_running = false;
    vision_init(&vision);
    pipeline_init();
    if (!arm_init(&arm, HAL_GetTick())) Error_Handler();
    rearm(&huart1, &rx1_byte, &rx1_rearm);
    rearm(&huart3, &rx3_byte, &rx3_rearm);
}

void App_Poll(void) {
    uint32_t now = HAL_GetTick();

    arm_tick(&arm, now);

    if (rx1_rearm) {
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        rx1_rearm = 0u;
        (void)HAL_UART_AbortReceive(&huart1);
        rearm(&huart1, &rx1_byte, &rx1_rearm);
        __set_PRIMASK(mask);
        arm_receive_fault(&arm);
    }
    if (rx3_rearm) {
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        rx3_rearm = 0u;
        (void)HAL_UART_AbortReceive(&huart3);
        rearm(&huart3, &rx3_byte, &rx3_rearm);
        __set_PRIMASK(mask);
        vision_init(&vision);
    }

    /* 限流处理，保证一串命令或视觉帧不会饿死运动插补。 */
    for (unsigned n = 0; n < 64u; ++n) {
        int c = ring_take(&ring1);
        if (c == -1) break;
        if (c == -2) { arm_receive_fault(&arm); break; }
        arm_receive(&arm, (uint8_t)c, HAL_GetTick());
    }

    for (unsigned n = 0; n < 64u; ++n) {
        vision_detection_t det;
        int c = ring_take(&ring3);
        if (c == -1) break;
        if (c == -2) { vision_init(&vision); break; }
        if (vision_feed(&vision, (uint8_t)c, HAL_GetTick(),
                        (uint16_t)ARM_DEFAULT_DEPTH_MM, &det)) {
            pipeline_on_detection(&arm, det.color, det.u, det.v, det.depth,
                                  HAL_GetTick());
        }
    }

    vision_poll(&vision, HAL_GetTick());
    pipeline_poll(&arm, HAL_GetTick());
}
