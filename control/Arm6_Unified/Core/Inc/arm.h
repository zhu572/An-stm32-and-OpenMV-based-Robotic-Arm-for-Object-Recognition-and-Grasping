#ifndef ARM_H
#define ARM_H
#include "protocol.h"
#include "arm_config.h"

typedef struct {
    protocol_t protocol;
    uint16_t current[ARM_SERVO_COUNT], start[ARM_SERVO_COUNT], target[ARM_SERVO_COUNT];
    uint16_t step, total_steps;
    uint32_t last_step_ms, last_valid_ms;
    bool enabled, moving, link_lost;
} arm_t;

bool arm_init(arm_t *a, uint32_t now);
void arm_receive(arm_t *a, uint8_t byte, uint32_t now);
void arm_tick(arm_t *a, uint32_t now);
void arm_receive_fault(arm_t *a);
uint16_t arm_angle_to_us(unsigned channel, uint16_t tenths);

/* --- 运动学桥接（供 pipeline 使用） -------------------------------------
 * 关节角与舵机逻辑角之间按 g_joint_map 换算；两者都用 deg，
 * 舵机侧内部以 0.1° 为单位存储。 */
void arm_joint_from_servo(const arm_t *a, float joint_deg[ARM_JOINT_COUNT]);
bool arm_set_joints(arm_t *a, const float joint_deg[ARM_JOINT_COUNT], uint32_t now);

/* Board adapter. Tests replace these functions with captures. */
void arm_port_pwm(unsigned channel, uint16_t pulse_us);
void arm_port_enable(bool enable);
void arm_port_reply(const char *text);
#endif
