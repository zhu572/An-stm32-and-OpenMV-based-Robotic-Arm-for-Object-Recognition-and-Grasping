#include "main.h"
/* USER CODE BEGIN Includes */
#include "app_hal.h"
/* USER CODE END Includes */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    App_Init();
    /* USER CODE END 2 */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        App_Poll();
    }
    /* USER CODE END 3 */
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    /* HSI 8 MHz / 2 * 16 = 64 MHz；不依赖外部晶振。 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2; /* APB1=32 MHz；TIM2/TIM3 时钟=64 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1; /* USART1=64 MHz */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void timer_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, unsigned count) {
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};
    static const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    timer->Instance = instance;
    timer->Init.Prescaler = 63u;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 19999u;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(timer, &master) != HAL_OK) Error_Handler();
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1500u;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    for (unsigned k = 0; k < count; ++k) {
        if (HAL_TIM_PWM_ConfigChannel(timer, &oc, channels[k]) != HAL_OK) Error_Handler();
    }
    HAL_TIM_MspPostInit(timer);
    /* PWM 在收到显式 $ON# 之前保持关闭。 */
}

static void MX_TIM2_Init(void) { timer_init(&htim2, TIM2, 4u); }
static void MX_TIM3_Init(void) { timer_init(&htim3, TIM3, 2u); }

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200u;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* OpenMV 视觉链路：USART3 / PB10(TX) PB11(RX)，与电脑的 USART1 分开。 */
static void MX_USART3_UART_Init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200u;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* 切到定时器复用功能前先把六路信号线拉低。 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
}

void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* 内部致命错误时关闭 PWM。这不是硬件急停。 */
    TIM2->CCER = 0u; TIM3->CCER = 0u;
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}
