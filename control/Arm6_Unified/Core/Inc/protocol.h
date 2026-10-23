#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdbool.h>
#include <stdint.h>
#include "arm_config.h"

/* 命令种类。前 8 个与旧版一致（顺序不可变，parse() 用下标查表）。 */
typedef enum {
    CMD_PING, CMD_STATUS, CMD_ON, CMD_OFF, CMD_STOP, CMD_HOME,
    CMD_MOVE, CMD_SINGLE,
    CMD_AUTO,    /* $AUTO,0# / $AUTO,1#              视觉闭环开关      */
    CMD_TARGET,  /* $TARGET,u,v[,depth]#              手动注入一次检测  */
    CMD_KIN      /* $KIN#                             查询运动学状态    */
} command_kind_t;

typedef struct {
    command_kind_t kind;
    uint16_t angles[ARM_SERVO_COUNT];
    uint16_t arg[3];   /* AUTO: arg[0]=0/1；TARGET: arg[0]=u, arg[1]=v, arg[2]=depth */
    uint8_t channel;
} arm_command_t;

typedef enum { RX_NONE, RX_COMMAND, RX_FORMAT, RX_TOO_LONG, RX_TIMEOUT } rx_result_t;

typedef struct {
    char frame[ARM_FRAME_CAPACITY];
    uint16_t length;
    uint32_t last_byte_ms;
    bool collecting;
} protocol_t;

void protocol_reset(protocol_t *p);
rx_result_t protocol_expire(protocol_t *p, uint32_t now);
rx_result_t protocol_feed(protocol_t *p, uint8_t byte, uint32_t now, arm_command_t *out);
#endif
