#include "arm_fk.h"
#include "kin_config.h"
#include <math.h>

void arm_fk(float j1, float j2, float j3, float j4,
            float *out_x, float *out_y, float *out_z)
{
    float r1 = KIN_D2R(j1);
    float r2 = KIN_D2R(j2);
    float r3 = KIN_D2R(j3);
    float theta = r2 + r3 + KIN_D2R(j4);
    /* 臂平面内的带符号径向 / 相对肩高 */
    float l = KIN_A2 * cosf(r2) + KIN_A3 * cosf(r2 + r3) + KIN_A4 * cosf(theta);
    float h = KIN_A2 * sinf(r2) + KIN_A3 * sinf(r2 + r3) + KIN_A4 * sinf(theta);

    *out_x = l * cosf(r1);
    *out_y = l * sinf(r1) - KIN_P;
    *out_z = KIN_A1 + h;
}

void arm_fk_tool_dir(float j1, float j2, float j3, float j4,
                     float *dx, float *dy, float *dz)
{
    float r1 = KIN_D2R(j1);
    float theta = KIN_D2R(j2) + KIN_D2R(j3) + KIN_D2R(j4);
    float c = cosf(theta);
    float s = sinf(theta);

    *dx = c * cosf(r1);
    *dy = c * sinf(r1);
    *dz = s;
}

void arm_fk_pose(float j1, float j2, float j3, float j4, Mat4 *T_base_ee)
{
    float r1 = KIN_D2R(j1);
    float r2 = KIN_D2R(j2);
    float r3 = KIN_D2R(j3);
    float theta = r2 + r3 + KIN_D2R(j4);
    float l, h, c, s;

    l = KIN_A2 * cosf(r2) + KIN_A3 * cosf(r2 + r3) + KIN_A4 * cosf(theta);
    h = KIN_A2 * sinf(r2) + KIN_A3 * sinf(r2 + r3) + KIN_A4 * sinf(theta);

    c = cosf(theta);
    s = sinf(theta);

    /* 旋转列：x = 臂平面内垂直于 z，y = z × x */
    T_base_ee->m[0][0] = -s * cosf(r1);
    T_base_ee->m[1][0] = -s * sinf(r1);
    T_base_ee->m[2][0] = c;

    T_base_ee->m[0][1] = sinf(r1);
    T_base_ee->m[1][1] = -cosf(r1);
    T_base_ee->m[2][1] = 0.0f;

    T_base_ee->m[0][2] = c * cosf(r1);
    T_base_ee->m[1][2] = c * sinf(r1);
    T_base_ee->m[2][2] = s;

    T_base_ee->m[0][3] = l * cosf(r1);
    T_base_ee->m[1][3] = l * sinf(r1) - KIN_P;
    T_base_ee->m[2][3] = KIN_A1 + h;

    T_base_ee->m[3][0] = 0.0f;
    T_base_ee->m[3][1] = 0.0f;
    T_base_ee->m[3][2] = 0.0f;
    T_base_ee->m[3][3] = 1.0f;
}
