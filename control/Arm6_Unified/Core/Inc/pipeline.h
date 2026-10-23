#ifndef PIPELINE_H
#define PIPELINE_H
#include "arm.h"

/* 视觉闭环流水线：
 *   视觉帧 (u,v,depth) → 当前 EE 位姿(由舵机角 FK) → 手眼链 → 基座系目标位姿
 *   → 闭式逆解 → 关节角 → 舵机逻辑角 → 现有插补 → PWM
 *
 * AUTO 关闭时视觉帧只计数不解算；解算受 ARM_KIN_MIN_INTERVAL_MS 限速，
 * 避免在 20 ms 插补周期里反复求解拖慢动作。
 */

void pipeline_init(void);

bool pipeline_auto_enabled(void);
void pipeline_set_auto(bool on);

/* 每轮主循环调用：AUTO 下校验互锁，掉线/未使能时自动退出闭环。 */
void pipeline_poll(arm_t *a, uint32_t now);

/* 收到一帧视觉检测（UART3）。 */
void pipeline_on_detection(arm_t *a, uint8_t color, uint8_t u, uint8_t v,
                           uint16_t depth, uint32_t now);

/* 手动注入一次检测（$TARGET,u,v[,d]#），无论 AUTO 是否开启都会解算。 */
bool pipeline_on_target(arm_t *a, uint16_t u, uint16_t v, uint16_t depth,
                        uint32_t now);

/* 回复 $KIN 状态行。 */
void pipeline_report(const arm_t *a);

#endif /* PIPELINE_H */
