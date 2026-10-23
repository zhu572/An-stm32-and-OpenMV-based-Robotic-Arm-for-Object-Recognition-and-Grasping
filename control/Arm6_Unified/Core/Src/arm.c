#include "arm.h"
#include "pipeline.h"
#include "kin_config.h"
#include <string.h>

/* 关节限位的数组形式，用于开机自检（映射必须落在舵机可达区间内）。 */
static const float k_joint_min[ARM_JOINT_COUNT] = {KIN_J1_MIN, KIN_J2_MIN, KIN_J3_MIN, KIN_J4_MIN};
static const float k_joint_max[ARM_JOINT_COUNT] = {KIN_J1_MAX, KIN_J2_MAX, KIN_J3_MAX, KIN_J4_MAX};

static bool joint_map_coherent(void) {
    for (unsigned k = 0; k < ARM_JOINT_COUNT; ++k) {
        const joint_map_t *m = &g_joint_map[k];
        float zero = (float)m->zero_tenths * 0.1f;
        float s0 = zero + (float)m->sign * k_joint_min[k];
        float s1 = zero + (float)m->sign * k_joint_max[k];
        float lo = s0 < s1 ? s0 : s1;
        float hi = s0 < s1 ? s1 : s0;
        float servo_lo = (float)g_servo_config[k].angle_min_tenths * 0.1f;
        float servo_hi = (float)g_servo_config[k].angle_max_tenths * 0.1f;
        if (lo < servo_lo - 0.05f || hi > servo_hi + 0.05f) return false;
    }
    return true;
}

static char *append_uint(char *p, unsigned value) {
    char reverse[10]; unsigned n = 0;
    do { reverse[n++] = (char)('0' + value % 10u); value /= 10u; } while (value != 0u);
    while (n != 0u) *p++ = reverse[--n];
    return p;
}

uint16_t arm_angle_to_us(unsigned channel, uint16_t tenths) {
    const servo_config_t *c = &g_servo_config[channel];
    uint32_t x = c->reverse ? 1800u - tenths : tenths;
    int32_t pulse = c->pulse_min_us + (int32_t)((x * (c->pulse_max_us - c->pulse_min_us) + 900u) / 1800u) + c->trim_us;
    if (pulse < c->pulse_min_us) pulse = c->pulse_min_us;
    if (pulse > c->pulse_max_us) pulse = c->pulse_max_us;
    return (uint16_t)pulse;
}

static void publish(const arm_t *a) {
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) arm_port_pwm(k, arm_angle_to_us(k, a->current[k]));
}

static void hold(arm_t *a) {
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) a->target[k] = a->current[k];
    a->moving = false;
}

bool arm_init(arm_t *a, uint32_t now) {
    memset(a, 0, sizeof(*a));
    arm_port_enable(false);
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) {
        const servo_config_t *c = &g_servo_config[k];
        if (c->pulse_min_us < 500u || c->pulse_max_us > 2500u || c->pulse_min_us >= c->pulse_max_us ||
            c->angle_min_tenths > c->angle_max_tenths || c->angle_max_tenths > 1800u ||
            c->home_tenths < c->angle_min_tenths || c->home_tenths > c->angle_max_tenths || c->reverse > 1u) {
            arm_port_reply("$ERR,CONFIG#\r\n"); return false;
        }
        a->current[k] = c->home_tenths; a->target[k] = c->home_tenths;
    }
    /* 关节点限位与舵机可达区间必须自洽，否则闭环每条指令都会撞 LIMIT。 */
    if (!joint_map_coherent()) {
        arm_port_reply("$ERR,CONFIG#\r\n"); return false;
    }
    a->last_valid_ms = now; a->last_step_ms = now;
    publish(a);
    arm_port_reply("$READY,ARM6,OFF#\r\n");
    return true;
}

static bool move(arm_t *a, const uint16_t values[ARM_SERVO_COUNT], uint32_t now) {
    unsigned distance = 0;
    if (!a->enabled || a->link_lost) { arm_port_reply("$ERR,DISABLED#\r\n"); return false; }
    /* Validate all six before committing ANY change. */
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) {
        if (values[k] < g_servo_config[k].angle_min_tenths || values[k] > g_servo_config[k].angle_max_tenths) {
            arm_port_reply("$ERR,LIMIT#\r\n"); return false;
        }
    }
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) {
        unsigned d = values[k] > a->current[k] ? values[k] - a->current[k] : a->current[k] - values[k];
        if (d > distance) distance = d;
        a->start[k] = a->current[k]; a->target[k] = values[k];
    }
    a->total_steps = (uint16_t)((distance + ARM_STEP_TENTHS - 1u) / ARM_STEP_TENTHS);
    a->step = 0; a->moving = distance != 0u; a->last_step_ms = now;
    arm_port_reply("$ACK,MOVE#\r\n");
    if (!a->moving) arm_port_reply("$DONE#\r\n");
    return true;
}

/* --- 关节角 ↔ 舵机逻辑角 ------------------------------------------------ */

void arm_joint_from_servo(const arm_t *a, float joint_deg[ARM_JOINT_COUNT]) {
    for (unsigned k = 0; k < ARM_JOINT_COUNT; ++k) {
        float servo_deg = (float)a->current[k] * 0.1f;
        float zero_deg = (float)g_joint_map[k].zero_tenths * 0.1f;
        joint_deg[k] = (servo_deg - zero_deg) * (float)g_joint_map[k].sign;
    }
}

bool arm_set_joints(arm_t *a, const float joint_deg[ARM_JOINT_COUNT], uint32_t now) {
    uint16_t next[ARM_SERVO_COUNT];
    if (!joint_deg) return false;
    /* 未参与运动学的 S5/S6 保持在当前设定值。 */
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) next[k] = a->current[k];
    for (unsigned k = 0; k < ARM_JOINT_COUNT; ++k) {
        float servo_deg = (float)g_joint_map[k].zero_tenths * 0.1f
                        + (float)g_joint_map[k].sign * joint_deg[k];
        int tenths = (int)(servo_deg * 10.0f + (servo_deg >= 0.0f ? 0.5f : -0.5f));
        /* 关节解算出舵机可达范围之外时整帧拒绝，绝不夹紧后勉强下发：
         * 夹紧会让机械臂走到一个与目标无关的姿态。 */
        if (tenths < 0 || tenths > 1800) {
            arm_port_reply("$ERR,LIMIT#\r\n");
            return false;
        }
        next[k] = (uint16_t)tenths;
    }
    return move(a, next, now);
}

/* --- 命令执行 ----------------------------------------------------------- */

static void execute(arm_t *a, const arm_command_t *c, uint32_t now) {
    uint16_t next[ARM_SERVO_COUNT];
    char text[128]; char *p; bool valid = true;
    switch (c->kind) {
    case CMD_PING: arm_port_reply("$PONG#\r\n"); break;
    case CMD_STATUS:
        /* CURRENT 是最近一次软件设定值，不是实测舵机位置。 */
        memcpy(text, "$STATE,", 7u); p = text + 7;
        p = append_uint(p, a->enabled); *p++ = ',';
        p = append_uint(p, a->moving); *p++ = ',';
        p = append_uint(p, a->link_lost);
        for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) { *p++ = ','; p = append_uint(p, a->current[k]); }
        memcpy(p, "#\r\n", 4u);
        arm_port_reply(text); break;
    case CMD_ON:
        if (!a->enabled) { publish(a); arm_port_enable(true); }
        a->enabled = true; a->link_lost = false;
        arm_port_reply("$ACK,ON#\r\n"); break;
    case CMD_OFF:
        hold(a); a->enabled = false; a->link_lost = false; arm_port_enable(false);
        pipeline_set_auto(false);
        arm_port_reply("$ACK,OFF#\r\n"); break;
    case CMD_STOP:
        hold(a); pipeline_set_auto(false);
        arm_port_reply("$ACK,STOP#\r\n"); break;
    case CMD_HOME:
        for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) next[k] = g_servo_config[k].home_tenths;
        valid = move(a, next, now); break;
    case CMD_MOVE:
        pipeline_set_auto(false);           /* 直接控制优先，退出视觉闭环 */
        valid = move(a, c->angles, now); break;
    case CMD_SINGLE:
        /* 隔离调试：其余关节冻结在当前设定值。 */
        pipeline_set_auto(false);
        for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) next[k] = a->current[k];
        next[c->channel] = c->angles[0]; valid = move(a, next, now); break;
    case CMD_AUTO:
        if (c->arg[0] != 0u && (!a->enabled || a->link_lost)) {
            arm_port_reply("$ERR,DISABLED#\r\n"); valid = false; break;
        }
        pipeline_set_auto(c->arg[0] != 0u);
        arm_port_reply(c->arg[0] != 0u ? "$ACK,AUTO,1#\r\n" : "$ACK,AUTO,0#\r\n");
        break;
    case CMD_TARGET:
        valid = pipeline_on_target(a, c->arg[0], c->arg[1], c->arg[2], now);
        break;
    case CMD_KIN:
        pipeline_report(a);
        break;
    default: valid = false; break;
    }
    if (valid) a->last_valid_ms = now;
}

void arm_receive(arm_t *a, uint8_t byte, uint32_t now) {
    arm_command_t command;
    if (protocol_expire(&a->protocol, now) == RX_TIMEOUT) arm_port_reply("$ERR,FRAME_TIMEOUT#\r\n");
    switch (protocol_feed(&a->protocol, byte, now, &command)) {
    case RX_COMMAND: execute(a, &command, now); break;
    case RX_FORMAT: arm_port_reply("$ERR,FORMAT#\r\n"); break;
    case RX_TOO_LONG: arm_port_reply("$ERR,TOO_LONG#\r\n"); break;
    default: break;
    }
}

void arm_receive_fault(arm_t *a) {
    protocol_reset(&a->protocol);
    hold(a); if (a->enabled) a->link_lost = true;
    pipeline_set_auto(false);
    arm_port_reply("$ERR,RX_LOSS#\r\n");
}

void arm_tick(arm_t *a, uint32_t now) {
    if (protocol_expire(&a->protocol, now) == RX_TIMEOUT) arm_port_reply("$ERR,FRAME_TIMEOUT#\r\n");
    if (a->enabled && !a->link_lost && (uint32_t)(now - a->last_valid_ms) >= ARM_LINK_TIMEOUT_MS) {
        hold(a); a->link_lost = true;
        pipeline_set_auto(false);
        arm_port_reply("$WARN,LINK_TIMEOUT,HOLD#\r\n");
    }
    if (!a->enabled || !a->moving || a->link_lost || (uint32_t)(now - a->last_step_ms) < ARM_UPDATE_MS) return;
    /* 每次调用最多前进一步：主循环慢只会让动作变慢，不会跳变。 */
    a->last_step_ms = now; ++a->step;
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) {
        int32_t delta = (int32_t)a->target[k] - a->start[k];
        a->current[k] = (uint16_t)((int32_t)a->start[k] + delta * a->step / a->total_steps);
    }
    publish(a);
    if (a->step >= a->total_steps) { a->moving = false; arm_port_reply("$DONE#\r\n"); }
}
