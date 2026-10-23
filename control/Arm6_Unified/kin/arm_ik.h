#ifndef ARM_IK_H
#define ARM_IK_H

#include "mat4.h"

/* 4 自由度极坐标臂的闭式逆解。
 *
 * 返回值： 0 成功；-1 超出几何可达域；-2 几何可达但无满足关节限位的解。
 * 失败时 4 个关节角输出均置为 KIN_IK_FAIL。
 *
 * arm_ik       仅位置：theta 在 [KIN_IK_SCAN_MIN, KIN_IK_SCAN_MAX] 粗扫，
 *              取最接近 ARM_HOME 的一支（仅用于离线/主机测试，固件主流程不用）。
 * arm_ik_pose  位置 + 接近方向：先做常数级候选枚举精确匹配方向；若目标 z 轴
 *              不在该点方位角的竖直平面内（4R 臂无法指向），退化为
 *              「位置精确 + 方向最小残差」，残差经 dir_err_rad 返回。
 *              相机的接近方向在该平面外时必然出现残差，这是几何限制而非 bug。
 */
int arm_ik(float x, float y, float z,
           float *j1, float *j2, float *j3, float *j4);

int arm_ik_pose(const Mat4 *T_base_target, float *dir_err_rad,
                float *j1, float *j2, float *j3, float *j4);

#endif /* ARM_IK_H */
