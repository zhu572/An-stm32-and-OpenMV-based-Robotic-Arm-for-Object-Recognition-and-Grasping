#ifndef _BusServo_Driver
#define _BusServo_Driver

#include "stm32f10x.h"                  // Device header

//初始化PA2的GPIO和配置为串口发送
void BusServo_Init(void);

//调用串口发送数据
static void SendChar(char ch);

//拆解信息，转化成ASCALL码的形式
static void SendUInt(uint16_t value, uint8_t digits);

//目前没什么用
static void WaitTxDone(void);

//作为中转调用Send Char和Send Unit将所有的数据发送
void BusServo_SetPulse(uint8_t id, uint16_t pulse, uint16_t time_ms);

//将角度转化为脉宽 同时收id和时间
void BusServo_SetAngleTime(uint8_t id, float deg, uint16_t time_ms);
//初始化机械臂的位置，将视像头对准桌面
void placeintial(void);
	
void catch1(void);

#endif
