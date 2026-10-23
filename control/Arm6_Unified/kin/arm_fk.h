#ifndef ARM_FK_H
#define ARM_FK_H

#include "mat4.h"

/* 4 自由度极坐标臂的正运动学。
 *
 * 模型（见 Docs/架构与接线.md）：
 *   j1       基座偏航，绕基座系 Z 轴；肩关节轴线在 (0, -KIN_P, KIN_A1)
 *   j2/j3/j4 串联俯仰关节，俯仰角全 0 时各连杆沿 +X（臂平面径向）
 *
 * 末端位姿约定：
 *   z 轴 = 接近方向（沿最后一节连杆，θ = j2+j3+j4）
 *   x 轴 = 臂平面内、垂直于 z
 *   y 轴 = z × x（右手系）
 *
 * 单位：角度 deg，长度 mm。
 */

/* 仅位置正解（与 PC 端原型接口一致）。 */
void arm_fk(float j1, float j2, float j3, float j4,
            float *out_x, float *out_y, float *out_z);

/* 全位姿正解：T_base_ee。 */
void arm_fk_pose(float j1, float j2, float j3, float j4, Mat4 *T_base_ee);

/* 末端接近方向的单位向量（EE 旋转矩阵第 3 列）。 */
void arm_fk_tool_dir(float j1, float j2, float j3, float j4,
                     float *dx, float *dy, float *dz);

#endif /* ARM_FK_H */
