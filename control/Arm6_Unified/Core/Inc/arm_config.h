#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H
#include <stdint.h>

/* ============================================================================
 * arm_config.h - 舵机路配置 + 关节映射（固件侧唯一来源）
 *
 * 注意：本文件只负责「舵机"路"」，运动学参数在 ../kin/kin_config.h。
 * 两者不要混用。
 * ============================================================================ */

#define ARM_SERVO_COUNT       6u
#define ARM_JOINT_COUNT       4u   /* j1..j4 由运动学驱动，S5/S6 由参数固定 */

#define ARM_UPDATE_MS         20u  /* 插补周期：20 ms → 50 Hz */
#define ARM_STEP_TENTHS       10u  /* 每周期最多 1°，即 50 °/s */
#define ARM_LINK_TIMEOUT_MS   2000u
#define ARM_FRAME_TIMEOUT_MS  200u
#define ARM_FRAME_CAPACITY    96u

/* 视觉闭环相关 */
#define ARM_DEFAULT_DEPTH_MM     200u  /* 视觉帧不带 depth 时的默认深度 [TEMP] */
#define ARM_KIN_MIN_INTERVAL_MS  150u  /* 两次解算最小间隔，避免拖慢插补 */
#define ARM_KIN_MAX_DIR_ERR_MRAD 350u  /* 允许的接近方向残差上限，mrad */

typedef struct {
    uint16_t pulse_min_us, pulse_max_us;
    uint16_t angle_min_tenths, angle_max_tenths, home_tenths;
    int16_t trim_us;
    uint8_t reverse;
} servo_config_t;

/* 关节角 ↔ 舵机逻辑角映射：
 *     servo_deg = zero_tenths/10 + sign * joint_deg
 *     joint_deg = (servo_deg - zero_tenths/10) * sign
 * sign 取 +1/-1；zero_tenths 是「关节 0° 对应的舵机角度」。[TEMP] 待实测。
 */
typedef struct {
    uint16_t zero_tenths;
    int8_t sign;
} joint_map_t;

extern const servo_config_t g_servo_config[ARM_SERVO_COUNT];
extern const joint_map_t g_joint_map[ARM_JOINT_COUNT];

#endif /* ARM_CONFIG_H */
