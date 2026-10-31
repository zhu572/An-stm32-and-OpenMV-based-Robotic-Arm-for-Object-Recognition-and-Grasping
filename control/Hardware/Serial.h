#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

/* USART1：PA9 = TX，PA10 = RX，115200 8N1
 *
 * OpenMV main.py 发送的帧（6 字节）：
 *   0x2C 0x12 color cx cy 0x5B
 *   color : 'B'=0x42 蓝 / 'Y'=0x59 黄 / 'G'=0x47 绿 / 'R'=0x52 红 / 'A'=0x41 无目标
 *   cx,cy : 色块中心坐标（QQVGA 160x120，单字节足够）
 *   0x5B  : 帧尾
 */
void     Serial_Init(void);

/* 取一帧解析结果：有新帧返回 1，没有返回 0 */
uint8_t  Serial_GetFrame(uint8_t *color, uint8_t *cx, uint8_t *cy);

/* 累计收到的字节数（调试用） */
uint32_t Serial_GetRxBytes(void);

#endif
