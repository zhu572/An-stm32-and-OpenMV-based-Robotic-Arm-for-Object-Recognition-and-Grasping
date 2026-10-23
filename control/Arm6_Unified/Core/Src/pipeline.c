#include "pipeline.h"
#include "kin_config.h"
#include "handeye.h"
#include "arm_fk.h"
#include "arm_ik.h"
#include "vision_frame.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * 视觉闭环流水线
 *
 *   (u,v,depth) ──反投影──▶ cam 系点 ──T_ee_cam──▶ EE 系 ──T_base_ee──▶ 基座系
 *                                                       ▲
 *                                    当前舵机设定角 ──FK─┘
 *   基座系目标位姿 ──arm_ik_pose()──▶ j1..j4 ──g_joint_map──▶ 舵机逻辑角
 *                                                     ──插补──▶ PWM
 *
 * 耗时控制：arm_ik_pose() 是常数级候选枚举（不是 1° 全扫描），
 * 但仍受 ARM_KIN_MIN_INTERVAL_MS 限速，保证 20 ms 插补周期不被拖慢。
 * --------------------------------------------------------------------------- */

static bool s_auto;
static uint32_t s_last_solve_ms;

static uint16_t s_frames;    /* 收到的视觉帧总数 */
static uint16_t s_solves;    /* 成功解算并下发动作的次数 */
static uint16_t s_rejects;   /* 解算被拒的次数 */
static uint16_t s_skipped;   /* 因限速被跳过的帧数 */

static uint8_t s_last_color;
static uint16_t s_last_u, s_last_v, s_last_depth;
static float s_last_err_rad;
static int16_t s_last_joint_tenths[ARM_JOINT_COUNT];
static bool s_has_solution;

static char *put_uint(char *p, unsigned value)
{
    char reverse[10];
    unsigned n = 0;
    do {
        reverse[n++] = (char)('0' + value % 10u);
        value /= 10u;
    } while (value != 0u);
    while (n != 0u) *p++ = reverse[--n];
    return p;
}

static char *put_int(char *p, int value)
{
    if (value < 0) {
        *p++ = '-';
        value = -value;
    }
    return put_uint(p, (unsigned)value);
}

static void reply_target_error(unsigned code)
{
    char text[24];
    char *p = text + 9;
    memcpy(text, "$TGT,ERR,", 9u);
    p = put_uint(p, code);
    memcpy(p, "#\r\n", 4u);
    arm_port_reply(text);
}

static void reply_target_ok(uint16_t u, uint16_t v, uint16_t depth)
{
    char text[96];
    char *p = text + 8;
    memcpy(text, "$TGT,OK,", 8u);
    p = put_uint(p, u); *p++ = ',';
    p = put_uint(p, v); *p++ = ',';
    p = put_uint(p, depth);
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        *p++ = ',';
        p = put_int(p, (int)s_last_joint_tenths[i]);
    }
    *p++ = ',';
    p = put_int(p, (int)(s_last_err_rad * 1000.0f + 0.5f));
    memcpy(p, "#\r\n", 4u);
    arm_port_reply(text);
}

/* 一次完整解算。成功返回 true；失败已回复 $TGT,ERR,n#。
 * 错误码：1 反投影失败，2 逆解失败/不可达，3 接近方向残差超限。 */
static bool solve(arm_t *a, uint16_t u, uint16_t v, uint16_t depth, uint32_t now)
{
    CamIntrinsics k;
    float joint_now[ARM_JOINT_COUNT];
    float joint[ARM_JOINT_COUNT];
    float derr = 0.0f;
    Mat4 T_ee, T_tgt;
    Pose target;
    int rc;

    k.fx = KIN_CAM_FX;
    k.fy = KIN_CAM_FY;
    k.cx = KIN_CAM_CX;
    k.cy = KIN_CAM_CY;

    s_last_solve_ms = now;
    s_last_u = u;
    s_last_v = v;
    s_last_depth = depth;

    /* 1. 当前 EE 位姿由「舵机设定角」而非实测角给出 —— 无编码器时这是唯一信息。 */
    arm_joint_from_servo(a, joint_now);
    arm_fk_pose(joint_now[0], joint_now[1], joint_now[2], joint_now[3], &T_ee);

    /* 2. 像素 + 深度 → 基座系目标位姿（T_ee_cam 用已安装标定） */
    if (handeye_pixel_to_base(&k, NULL, &T_ee, (float)u, (float)v, (float)depth,
                              &target) != 0) {
        ++s_rejects;
        reply_target_error(1u);
        return false;
    }

    /* 3. 位姿逆解 */
    mat4_from_pose(&target, &T_tgt);
    rc = arm_ik_pose(&T_tgt, &derr, &joint[0], &joint[1], &joint[2], &joint[3]);
    if (rc != 0) {
        ++s_rejects;
        reply_target_error(2u);
        return false;
    }

    /* 4. 接近方向残差：4R 臂的 z 轴必须落在目标点方位角的竖直平面内，
     *    相机横向偏置或目标偏出该平面时必然有残差。超限视为不可抓。 */
    if (derr * 1000.0f > (float)ARM_KIN_MAX_DIR_ERR_MRAD) {
        ++s_rejects;
        reply_target_error(3u);
        return false;
    }

    /* 5. 关节角 → 舵机角，交给现有插补（限位失败会自己回 $ERR,LIMIT#） */
    if (!arm_set_joints(a, joint, now)) {
        ++s_rejects;
        return false;
    }

    s_last_err_rad = derr;
    s_has_solution = true;
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        s_last_joint_tenths[i] =
            (int16_t)(joint[i] * 10.0f + (joint[i] >= 0.0f ? 0.5f : -0.5f));
    }
    ++s_solves;
    reply_target_ok(u, v, depth);
    return true;
}

void pipeline_init(void)
{
    s_auto = false;
    s_last_solve_ms = 0u;
    s_frames = 0u;
    s_solves = 0u;
    s_rejects = 0u;
    s_skipped = 0u;
    s_last_color = 0u;
    s_last_u = 0u;
    s_last_v = 0u;
    s_last_depth = 0u;
    s_last_err_rad = 0.0f;
    s_has_solution = false;
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) s_last_joint_tenths[i] = 0;
}

bool pipeline_auto_enabled(void)
{
    return s_auto;
}

void pipeline_set_auto(bool on)
{
    if (on && !s_auto) s_last_solve_ms = 0u;   /* 允许立即解算一次 */
    s_auto = on;
}

void pipeline_poll(arm_t *a, uint32_t now)
{
    (void)now;
    /* 未使能或链路掉线时视觉闭环必须失效，不能靠残留状态继续发动作。 */
    if (s_auto && (!a->enabled || a->link_lost)) s_auto = false;
}

void pipeline_on_detection(arm_t *a, uint8_t color, uint8_t u, uint8_t v,
                           uint16_t depth, uint32_t now)
{
    ++s_frames;
    s_last_color = color;

    /* 'A' = 画面里没有目标的心跳帧：只计帧数，绝不当成目标解算。
     * 否则 OpenMV 一直发送时，固件会对着一个幻影像素点反复规划动作。 */
    if (color == VISION_COLOR_NONE) return;

    if (!s_auto) return;
    if (!a->enabled || a->link_lost) { s_auto = false; return; }
    if ((uint32_t)(now - s_last_solve_ms) < ARM_KIN_MIN_INTERVAL_MS) {
        ++s_skipped;
        return;
    }
    (void)solve(a, u, v, depth, now);
}

bool pipeline_on_target(arm_t *a, uint16_t u, uint16_t v, uint16_t depth,
                        uint32_t now)
{
    if (!a->enabled || a->link_lost) {
        arm_port_reply("$ERR,DISABLED#\r\n");
        return false;
    }
    return solve(a, u, v, depth, now);
}

void pipeline_report(const arm_t *a)
{
    char text[128];
    char *p = text + 5;

    (void)a;
    memcpy(text, "$KIN,", 5u);
    p = put_uint(p, s_auto ? 1u : 0u); *p++ = ',';
    p = put_uint(p, (unsigned)s_frames); *p++ = ',';
    p = put_uint(p, (unsigned)s_solves); *p++ = ',';
    p = put_uint(p, (unsigned)s_rejects); *p++ = ',';
    p = put_uint(p, (unsigned)s_skipped); *p++ = ',';
    p = put_uint(p, (unsigned)s_last_color); *p++ = ',';
    p = put_uint(p, (unsigned)s_last_u); *p++ = ',';
    p = put_uint(p, (unsigned)s_last_v); *p++ = ',';
    p = put_uint(p, (unsigned)s_last_depth);
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        *p++ = ',';
        p = put_int(p, (int)s_last_joint_tenths[i]);
    }
    *p++ = ',';
    p = put_int(p, s_has_solution ? (int)(s_last_err_rad * 1000.0f + 0.5f) : -1);
    memcpy(p, "#\r\n", 4u);
    arm_port_reply(text);
}
