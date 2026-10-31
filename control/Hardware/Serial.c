#include "Serial.h"

/* 帧格式：0x2C 0x12 color cx cy 0x5B */
#define FRAME_LEN    6
#define FRAME_HEAD1  0x2C
#define FRAME_HEAD2  0x12
#define FRAME_TAIL   0x5B

static uint8_t s_buf[FRAME_LEN];
static uint8_t s_idx = 0;

static volatile uint8_t  s_color = 0;
static volatile uint8_t  s_cx = 0;
static volatile uint8_t  s_cy = 0;
static volatile uint8_t  s_ready = 0;
static volatile uint32_t s_rx_bytes = 0;

void Serial_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	/* PA9 = USART1_TX，复用推挽输出 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA10 = USART1_RX，浮空输入 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

/* 每收到一个字节进一次中断：状态机找帧头，收满 6 字节后校验帧尾 */
void USART1_IRQHandler(void)
{
	uint8_t ch;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		ch = (uint8_t)USART_ReceiveData(USART1);
		s_rx_bytes++;

		if (s_idx == 0)
		{
			if (ch == FRAME_HEAD1)
			{
				s_buf[s_idx++] = ch;
			}
		}
		else if (s_idx == 1)
		{
			if (ch == FRAME_HEAD2)      s_buf[s_idx++] = ch;
			else if (ch == FRAME_HEAD1) s_idx = 1;   /* 还是 0x2C，继续等 0x12 */
			else                        s_idx = 0;
		}
		else
		{
			s_buf[s_idx++] = ch;
			if (s_idx >= FRAME_LEN)
			{
				s_idx = 0;
				if (s_buf[FRAME_LEN - 1] == FRAME_TAIL)
				{
					s_color = s_buf[2];
					s_cx    = s_buf[3];
					s_cy    = s_buf[4];
					s_ready = 1;
				}
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

uint8_t Serial_GetFrame(uint8_t *color, uint8_t *cx, uint8_t *cy)
{
	if (s_ready)
	{
		*color = s_color;
		*cx    = s_cx;
		*cy    = s_cy;
		s_ready = 0;
		return 1;
	}
	return 0;
}

uint32_t Serial_GetRxBytes(void)
{
	return s_rx_bytes;
}
