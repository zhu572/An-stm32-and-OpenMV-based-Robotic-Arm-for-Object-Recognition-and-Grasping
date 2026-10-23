#include "arm_ik.h"
#include "arm_fk.h"
#include "kin_config.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * 闭式逆解（MCU 版）
 *
 * 几何：末端在肩关节的（方位 r1、带符号径向 L、高度 H）坐标下满足
 *     L = a2*cos(j2) + a3*cos(j2+j3) + a4*cos(theta)
 *     H = a2*sin(j2) + a3*sin(j2+j3) + a4*sin(theta)     theta = j2+j3+j4
 * 给定 (L, H, theta) 后腕点 w = (L - a4*cos(theta), H - a4*sin(theta))，
 * (j2, j3) 由平面 2R 闭式解给出：
 *     cos(j3) = (|w|^2 - a2^2 - a3^2) / (2*a2*a3)
 *     j2      = atan2(wy, wx) - atan2(a3*sin(j3), a2 + a3*cos(j3))
 *     j4      = theta - j2 - j3
 *
 * 与 PC 端 double 版的差异：
 *   PC 版对 theta 做 1° 全范围扫描来找最优接近方向。本版利用解析结论
 *   「给定 r1 时，使 dot(z_tool, z_target) 最大的 theta 恰为
 *     theta0 = atan2(z_z, z_x*cos(r1) + z_y*sin(r1))」，
 *   因此 phase 2 只需常数个候选即可，避免 540 步扫描在 72 MHz M3 上
 *   超时。仅当常数候选因关节限位全部不可行时，才回退到粗扫描（phase 3）。
 * --------------------------------------------------------------------------- */

typedef struct {
    float j1, j2, j3, j4;
} kin_joints;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void set_fail(float *j1, float *j2, float *j3, float *j4)
{
    if (j1) *j1 = KIN_IK_FAIL;
    if (j2) *j2 = KIN_IK_FAIL;
    if (j3) *j3 = KIN_IK_FAIL;
    if (j4) *j4 = KIN_IK_FAIL;
}

static int in_limits(const kin_joints *j)
{
    return (j->j1 >= KIN_J1_MIN - KIN_IK_LIMIT_EPS &&
            j->j1 <= KIN_J1_MAX + KIN_IK_LIMIT_EPS &&
            j->j2 >= KIN_J2_MIN - KIN_IK_LIMIT_EPS &&
            j->j2 <= KIN_J2_MAX + KIN_IK_LIMIT_EPS &&
            j->j3 >= KIN_J3_MIN - KIN_IK_LIMIT_EPS &&
            j->j3 <= KIN_J3_MAX + KIN_IK_LIMIT_EPS &&
            j->j4 >= KIN_J4_MIN - KIN_IK_LIMIT_EPS &&
            j->j4 <= KIN_J4_MAX + KIN_IK_LIMIT_EPS);
}

static void normalize_j1(kin_joints *j)
{
    while (j->j1 > 180.0f) j->j1 -= 360.0f;
    while (j->j1 <= -180.0f) j->j1 += 360.0f;
}

/* 确定性选支：取最接近 home 的解。 */
static float home_cost(const kin_joints *j)
{
    float d1 = j->j1 - KIN_HOME_J1;
    float d2 = j->j2 - KIN_HOME_J2;
    float d3 = j->j3 - KIN_HOME_J3;
    float d4 = j->j4 - KIN_HOME_J4;
    return d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4;
}

/* 腕点 (wx, wy) 的平面 2R 子问题。返回肘分支数 0..2，角度为弧度。 */
static int solve_2r(float wx, float wy, float *j2_out, float *j3_out)
{
    float r2 = wx * wx + wy * wy;
    float c3 = (r2 - KIN_A2 * KIN_A2 - KIN_A3 * KIN_A3) / (2.0f * KIN_A2 * KIN_A3);
    float s3, gamma;
    int n = 0, b;

    if (c3 < -1.0f - KIN_IK_LIMIT_EPS || c3 > 1.0f + KIN_IK_LIMIT_EPS) return 0;
    c3 = clampf(c3, -1.0f, 1.0f);
    s3 = sqrtf(1.0f - c3 * c3);
    gamma = atan2f(wy, wx);

    for (b = 0; b < 2; ++b) {
        float sj = (b == 0) ? s3 : -s3;
        float j3 = atan2f(sj, c3);
        float delta = atan2f(KIN_A3 * sj, KIN_A2 + KIN_A3 * c3);
        j2_out[n] = gamma - delta;
        j3_out[n] = j3;
        ++n;
        if (sj == 0.0f) break;   /* 奇异位形：两肘分支重合 */
    }
    return n;
}

/* 对一种 (r1, 带符号 L, theta) 表示枚举候选，逐个用 FK 回代校验。
 * *reach 在 2R 子问题可解时置 1（几何可达，即使被限位剔除）。 */
static int gen_frame(float r1, float l, float h, float theta,
                     float x, float y, float z,
                     int *reach, kin_joints *out, int max_out)
{
    float wx = l - KIN_A4 * cosf(theta);
    float wy = h - KIN_A4 * sinf(theta);
    float j2r[2], j3r[2];
    int nb = solve_2r(wx, wy, j2r, j3r);
    int n = 0, b;

    if (nb > 0) *reach = 1;

    for (b = 0; b < nb && n < max_out; ++b) {
        kin_joints c;
        float xc, yc, zc;

        c.j1 = KIN_R2D(r1);
        c.j2 = KIN_R2D(j2r[b]);
        c.j3 = KIN_R2D(j3r[b]);
        c.j4 = KIN_R2D(theta - j2r[b] - j3r[b]);
        normalize_j1(&c);

        if (!in_limits(&c)) continue;

        arm_fk(c.j1, c.j2, c.j3, c.j4, &xc, &yc, &zc);
        if (fabsf(xc - x) > KIN_IK_POSE_TOL ||
            fabsf(yc - y) > KIN_IK_POSE_TOL ||
            fabsf(zc - z) > KIN_IK_POSE_TOL) {
            continue;
        }

        out[n++] = c;
    }
    return n;
}

/* 目标点 → 肩坐标系 (r1, L, H)。 */
static void to_shoulder_frame(float x, float y, float z,
                              float *r1, float *l, float *h)
{
    float xr = x;
    float yr = y + KIN_P;
    float a = atan2f(yr, xr);

    *r1 = a;
    *l = xr * cosf(a) + yr * sinf(a);
    *h = z - KIN_A1;
}

/* 候选的接近方向与目标 z 轴夹角（弧度）。 */
static float dir_error_rad(const kin_joints *j, float zx, float zy, float zz)
{
    float dx, dy, dz, dot;

    arm_fk_tool_dir(j->j1, j->j2, j->j3, j->j4, &dx, &dy, &dz);
    dot = dx * zx + dy * zy + dz * zz;
    dot = clampf(dot, -1.0f, 1.0f);
    return acosf(dot);
}

int arm_ik_pose(const Mat4 *T, float *dir_err_rad,
                float *j1_out, float *j2_out, float *j3_out, float *j4_out)
{
    float x, y, z, zx, zy, zz;
    float r1, l, h, theta0, t;
    kin_joints best, cand[2];
    int have_best = 0, reach = 0;
    float best_cost = 0.0f, best_err = 0.0f;
    int f, i;

    set_fail(j1_out, j2_out, j3_out, j4_out);
    if (dir_err_rad) *dir_err_rad = -1.0f;
    if (!T) return -1;

    x = T->m[0][3];
    y = T->m[1][3];
    z = T->m[2][3];
    zx = T->m[0][2];
    zy = T->m[1][2];
    zz = T->m[2][2];

    to_shoulder_frame(x, y, z, &r1, &l, &h);

    /* 目标 z 轴在该点竖直平面内的仰角 */
    theta0 = atan2f(zz, zx * cosf(r1) + zy * sinf(r1));

    /* phase 1：精确匹配接近方向。
     * 同一物理点有 (r1, L) 与 (r1+PI, -L) 两种等价表示，theta 亦对应
     * theta0 与 PI-theta0；再叠 θ±2π 以覆盖限位边缘。 */
    for (f = 0; f < 6; ++f) {
        float rr = (f < 3) ? r1 : r1 + KIN_PI;
        float ll = (f < 3) ? l : -l;
        float th = (f < 3) ? theta0 : KIN_PI - theta0;
        int n;

        if (f % 3 == 1) th += KIN_TWO_PI;
        if (f % 3 == 2) th -= KIN_TWO_PI;

        n = gen_frame(rr, ll, h, th, x, y, z, &reach, cand, 2);
        for (i = 0; i < n; ++i) {
            float e = dir_error_rad(&cand[i], zx, zy, zz);
            if (e > KIN_IK_DIR_TOL) continue;
            {
                float cost = home_cost(&cand[i]);
                if (!have_best || cost < best_cost) {
                    best = cand[i];
                    best_cost = cost;
                    have_best = 1;
                }
            }
        }
    }

    if (have_best) {
        if (dir_err_rad) *dir_err_rad = dir_error_rad(&best, zx, zy, zz);
        if (j1_out) *j1_out = best.j1;
        if (j2_out) *j2_out = best.j2;
        if (j3_out) *j3_out = best.j3;
        if (j4_out) *j4_out = best.j4;
        return 0;
    }

    /* phase 2：最佳努力。theta0 已是该 r1 下方向残差最小的 theta
     * （gripper 竖直平面内投影），保留位置精确、报告残差。 */
    have_best = 0;
    best_err = 1.0e9f;
    best_cost = 0.0f;
    for (f = 0; f < 2; ++f) {
        float rr = (f == 0) ? r1 : r1 + KIN_PI;
        float ll = (f == 0) ? l : -l;
        int n = gen_frame(rr, ll, h, theta0, x, y, z, &reach, cand, 2);

        for (i = 0; i < n; ++i) {
            float e = dir_error_rad(&cand[i], zx, zy, zz);
            float cost = home_cost(&cand[i]);
            if (!have_best ||
                e < best_err - 1.0e-6f ||
                (e < best_err + 1.0e-6f && cost < best_cost)) {
                best = cand[i];
                best_err = e;
                best_cost = cost;
                have_best = 1;
            }
        }
    }

    /* phase 3：theta0 候选被限位全部剔除时的兜底粗扫描。
     * 正常路径不会走到这里，仅防止极端限位下无解。 */
    if (!have_best) {
        for (f = 0; f < 2; ++f) {
            float rr = (f == 0) ? r1 : r1 + KIN_PI;
            float ll = (f == 0) ? l : -l;

            for (t = KIN_IK_SCAN_MIN; t <= KIN_IK_SCAN_MAX + 1.0e-3f;
                 t += KIN_IK_COARSE_STEP) {
                int n = gen_frame(rr, ll, h, KIN_D2R(t), x, y, z,
                                  &reach, cand, 2);
                for (i = 0; i < n; ++i) {
                    float e = dir_error_rad(&cand[i], zx, zy, zz);
                    float cost = home_cost(&cand[i]);
                    if (!have_best ||
                        e < best_err - 1.0e-6f ||
                        (e < best_err + 1.0e-6f && cost < best_cost)) {
                        best = cand[i];
                        best_err = e;
                        best_cost = cost;
                        have_best = 1;
                    }
                }
            }
        }
    }

    if (!have_best) return reach ? -2 : -1;

    if (dir_err_rad) *dir_err_rad = best_err;
    if (j1_out) *j1_out = best.j1;
    if (j2_out) *j2_out = best.j2;
    if (j3_out) *j3_out = best.j3;
    if (j4_out) *j4_out = best.j4;
    return 0;
}

int arm_ik(float x, float y, float z,
           float *j1_out, float *j2_out, float *j3_out, float *j4_out)
{
    float r1, l, h, t;
    kin_joints best, cand[2];
    int have_best = 0, reach = 0;
    float best_cost = 0.0f;
    int f, i;

    set_fail(j1_out, j2_out, j3_out, j4_out);

    to_shoulder_frame(x, y, z, &r1, &l, &h);

    for (f = 0; f < 2; ++f) {
        float rr = (f == 0) ? r1 : r1 + KIN_PI;
        float ll = (f == 0) ? l : -l;

        for (t = KIN_IK_SCAN_MIN; t <= KIN_IK_SCAN_MAX + 1.0e-3f;
             t += KIN_IK_COARSE_STEP) {
            int n = gen_frame(rr, ll, h, KIN_D2R(t), x, y, z,
                              &reach, cand, 2);
            for (i = 0; i < n; ++i) {
                float cost = home_cost(&cand[i]);
                if (!have_best || cost < best_cost) {
                    best = cand[i];
                    best_cost = cost;
                    have_best = 1;
                }
            }
        }
    }

    if (!have_best) return reach ? -2 : -1;

    if (j1_out) *j1_out = best.j1;
    if (j2_out) *j2_out = best.j2;
    if (j3_out) *j3_out = best.j3;
    if (j4_out) *j4_out = best.j4;
    return 0;
}
