#include "stm32f10x.h"                  // Device header

#define BUS_USART      USART2
#define BUS_GPIO       GPIOA
#define BUS_TX_PIN     GPIO_Pin_2         
#define BUS_BAUD       115200U

void BusServo_Init()
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode =  GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = BUS_TX_PIN ;
	gpio.GPIO_Speed  = GPIO_Speed_50MHz;
	GPIO_Init(BUS_GPIO,&gpio);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitTypeDef usart;
	usart.USART_BaudRate            = BUS_BAUD;
  usart.USART_WordLength          = USART_WordLength_8b;
  usart.USART_StopBits            = USART_StopBits_1;
	usart.USART_Parity              = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode                = USART_Mode_Tx;   
  USART_Init(BUS_USART, &usart);

  USART_Cmd(BUS_USART, ENABLE);
}

static void SendChar(char ch)
{
    while (USART_GetFlagStatus(BUS_USART, USART_FLAG_TXE) == RESET) { }
    USART_SendData(BUS_USART, (uint16_t)(uint8_t)ch);
}


static void SendUInt(uint16_t value, uint8_t digits)
{
    char buf[5];
    uint8_t i;

    for (i = 0; i < digits; i++) {
        buf[digits - 1 - i] = (char)('0' + value % 10U);
        value /= 10U;
    }
    for (i = 0; i < digits; i++) {
        SendChar(buf[i]);
    }
}


static void WaitTxDone(void)
{
    while (USART_GetFlagStatus(BUS_USART, USART_FLAG_TC) == RESET) { }
		
}

void BusServo_SetPulse(uint8_t id, uint16_t pulse, uint16_t time_ms)
{
    if (pulse   < 500U)  { pulse   = 500U;   }   /* 软限位，防止撞机械限位 */
    if (pulse   > 2500U) { pulse   = 2500U;}
    if (time_ms > 9999U)  {time_ms = 9999U;}

    SendChar('#');
    SendUInt(id, 3);
    SendChar('P');
    SendUInt(pulse, 4);
    SendChar('T');
    SendUInt(time_ms, 4);
    SendChar('!');
    WaitTxDone();
}

/* 角度 → 脉宽：0° = 500us，180° = 2500us，90° = 1500us */
void BusServo_SetAngleTime(uint8_t id, float deg, uint16_t time_ms)
{
    float pulse0,pulse1,pulse2,pulse3,pulse4,pulse5;
		float deg0,deg1,deg2,deg3,deg4,deg5;
	
		if(id==0)
	{
		deg0=deg;
	if (deg0 < 0.0f)   deg0 = 0.0f;
  if (deg0 > 180.0f) deg0 = 180.0f;
  pulse0 = 1000.0f + deg0*(1000.0f/180.0f);
  BusServo_SetPulse(0, (uint16_t)(pulse0 + 0.5f), time_ms);
	}
	

	if(id==1||id==2||id==3||id==4)
	{
		deg1=deg;
	if (deg1 < 0.0f)   deg1 = 0.0f;
  if (deg1 > 225.0f) deg1 = 225.0f;
  pulse1 = 850.0f + deg1*(1300.0f/180.0f);
  BusServo_SetPulse(id, (uint16_t)(pulse1 + 0.5f), time_ms);
	}

		if(id==5)
	{
		deg2=deg;
	if (deg2 < 60.0f)   deg2 = 60.0f;//60度的时候就张开到最大，并不成比例
		if (deg2 > 120.0f) deg2 = 120.0f;//爪子的机械结构，120度的时候就完全比闭合
  pulse2 = 1000.0f + deg2*(1000.0f/180.0f);
  BusServo_SetPulse(5, (uint16_t)(pulse2 + 0.5f), time_ms);
	}
	
}




void placeintial()
{
	
	int intialpause[6] = {90,135,180,0,90,90 };
	int i;	
	for (i = 0; i <= 5; i++)
	{
		BusServo_SetAngleTime(i,intialpause[i],1000);
		Delay_ms(200);
	}
}

void catch1()
{
	int intialpause[6] = {90,90,180,15,90,90 };
	int i;	
	for (i = 0; i <= 5; i++)
	{
		BusServo_SetAngleTime(i,intialpause[i],1000);
		Delay_ms(200);
	}

}

