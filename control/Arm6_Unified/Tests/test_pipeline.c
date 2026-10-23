#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "arm.h"
#include "arm_config.h"
#include "protocol.h"
#include "vision_frame.h"
#include "pipeline.h"
#include "kin_config.h"

/* 主机侧：视觉帧解析 + 关节映射 + 视觉闭环 gating 自检。
 * 不参与 Keil 固件编译；运行：python Tools/test_host.py
 * 注意：本文件在交付环境中未经编译运行，首次运行如有问题请回传输出。 */

static char replies[65536];
static size_t used;
static uint16_t pulses[ARM_SERVO_COUNT];
static bool enabled;

void arm_port_pwm(unsigned channel, uint16_t pulse) {
    if (channel < ARM_SERVO_COUNT) pulses[channel] = pulse;
}
void arm_port_enable(bool on) { enabled = on; }
void arm_port_reply(const char *text) {
    size_t n = strlen(text);
    if (used + n + 1 >= sizeof(replies)) used = 0;
    memcpy(replies + used, text, n + 1);
    used += n;
}

static void clean_log(void) { used = 0; replies[0] = 0; }
static int has(const char *text) { return strstr(replies, text) != NULL; }
static void send(arm_t *a, const char *s, uint32_t now) {
    while (*s) arm_receive(a, (uint8_t)*s++, now);
}

static int feed(vision_parser_t *p, const uint8_t *bytes, unsigned n,
                uint32_t now, vision_detection_t *d) {
    int got = 0;
    for (unsigned i = 0; i < n; ++i) {
        if (vision_feed(p, bytes[i], now, (uint16_t)ARM_DEFAULT_DEPTH_MM, d)) got = 1;
    }
    return got;
}

/* 把舵机设定值直接摆好（绕过插补，用于构造确定的初始位姿）。 */
static void set_servo(arm_t *a, const uint16_t v[ARM_SERVO_COUNT]) {
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) a->current[k] = v[k];
    a->moving = false;
    a->target[0] = v[0]; a->target[1] = v[1]; a->target[2] = v[2];
    a->target[3] = v[3]; a->target[4] = v[4]; a->target[5] = v[5];
}

int main(void) {
    arm_t a;
    vision_parser_t vp;
    vision_detection_t det;
    float j[ARM_JOINT_COUNT];

    /* ---------------- 1. 视觉帧解析 ---------------- */
    vision_init(&vp);
    {
        static const uint8_t f6[] = {0x2Cu, 0x12u, 'B', 80u, 60u, 0x5Bu};
        static const uint8_t f7[] = {0x2Cu, 0x12u, 'R', 12u, 200u, 150u, 0x5Bu};
        static const uint8_t ftail[] = {0x2Cu, 0x12u, 'Y', 0x5Bu, 60u, 0x5Bu};
        static const uint8_t fnoise[] = {0x00u, 0xFFu, 0x2Cu, 0x99u,
                                         0x2Cu, 0x12u, 'G', 10u, 20u, 0x5Bu};

        memset(&det, 0, sizeof(det));
        assert(feed(&vp, f6, sizeof(f6), 0u, &det));
        assert(det.color == 'B' && det.u == 80u && det.v == 60u);
        assert(det.depth == ARM_DEFAULT_DEPTH_MM && det.payload_len == 3u);

        assert(feed(&vp, f7, sizeof(f7), 10u, &det));
        assert(det.color == 'R' && det.u == 12u && det.v == 200u);
        assert(det.depth == 150u && det.payload_len == 4u);

        /* 像素值本身等于 0x5B 时不能被当成帧尾 */
        assert(feed(&vp, ftail, sizeof(ftail), 20u, &det));
        assert(det.color == 'Y' && det.u == 0x5Bu && det.v == 60u);
        assert(det.depth == ARM_DEFAULT_DEPTH_MM);

        /* 帧外噪声必须被忽略 */
        assert(feed(&vp, fnoise, sizeof(fnoise), 30u, &det));
        assert(det.color == 'G' && det.u == 10u && det.v == 20u);
        puts("PASS: vision frame decode (6/7 byte, 0x5B payload, noise)");

        /* 半帧超时必须回收 */
        {
            static const uint8_t half[] = {0x2Cu, 0x12u, 'B'};
            uint16_t before = vp.drops;
            assert(!feed(&vp, half, sizeof(half), 100u, &det));
            vision_poll(&vp, 100u + VISION_TIMEOUT_MS + 1u);
            assert(vp.drops == (uint16_t)(before + 1u));
            printf("    frames=%u drops=%u\n", (unsigned)vp.frames, (unsigned)vp.drops);
            puts("PASS: half frame timeout is reclaimed");
        }
    }

    /* ---------------- 2. 关节角 ↔ 舵机角 ---------------- */
    assert(arm_init(&a, 0u));       /* 同时验证 g_joint_map 与限位自洽 */
    assert(!enabled);
    arm_joint_from_servo(&a, j);
    /* 初始 home = 90°，但 j2/j3 零位是 0°、j1/j4 零位是 90° */
    assert(fabsf(j[0] - 0.0f) < 1.0e-4f);
    assert(fabsf(j[1] - 90.0f) < 1.0e-4f);
    assert(fabsf(j[2] - 90.0f) < 1.0e-4f);
    assert(fabsf(j[3] - 0.0f) < 1.0e-4f);
    a.current[0] = 1000u;                 /* 舵机 100.0° → j1 = +10 */
    arm_joint_from_servo(&a, j);
    assert(fabsf(j[0] - 10.0f) < 1.0e-4f);
    a.current[0] = 800u;                  /* 舵机 80.0° → j1 = -10 */
    arm_joint_from_servo(&a, j);
    assert(fabsf(j[0] + 10.0f) < 1.0e-4f);
    puts("PASS: joint <-> servo mapping follows g_joint_map");

    /* ---------------- 3. AUTO 互锁 ---------------- */
    clean_log();
    send(&a, "$AUTO,1#", 10u);
    assert(!pipeline_auto_enabled());
    assert(has("$ERR,DISABLED#"));

    send(&a, "$ON#", 11u);
    assert(has("$ACK,ON#"));
    clean_log();
    send(&a, "$AUTO,1#", 12u);
    assert(pipeline_auto_enabled());
    assert(has("$ACK,AUTO,1#"));
    puts("PASS: AUTO requires an enabled arm");

    /* ---------------- 4. 视觉闭环成功下发 ---------------- */
    /* 折叠高位位姿 j=(0,60,90,-90) → 舵机 (90.0,60.0,90.0,0.0)，
     * 相机朝上偏前，画面中心 40 mm 处落在可达空间内（见 Docs/架构与接线.md） */
    {
        static const uint16_t folded[ARM_SERVO_COUNT] = {900u, 600u, 900u, 0u, 900u, 900u};
        set_servo(&a, folded);
        clean_log();
        pipeline_on_detection(&a, 'B', (uint8_t)KIN_CAM_CX, (uint8_t)KIN_CAM_CY,
                              40u, 1000u);
        assert(has("$TGT,OK"));
        assert(a.moving);
        assert(a.target[1] != 600u);      /* S2 必须被重新给定 */
        printf("    target S1..S4 = %u,%u,%u,%u\n",
               (unsigned)a.target[0], (unsigned)a.target[1],
               (unsigned)a.target[2], (unsigned)a.target[3]);
        puts("PASS: reachable detection produces a motion command");
    }

    /* ---------------- 5. 不可达目标必须被拒绝且不动 ---------------- */
    {
        uint16_t t1 = a.target[1];
        clean_log();
        pipeline_on_detection(&a, 'R', (uint8_t)KIN_CAM_CX, (uint8_t)KIN_CAM_CY,
                              255u, 1200u);
        assert(has("$TGT,ERR,2#"));
        assert(a.target[1] == t1);
        puts("PASS: unreachable target is rejected without touching the setpoints");
    }

    /* ---------------- 5b. 'A' 心跳帧只计数，不许解算 ---------------- */
    {
        uint16_t tx = a.target[0], t1 = a.target[1];
        clean_log();
        pipeline_on_detection(&a, 'A', (uint8_t)KIN_CAM_CX, (uint8_t)KIN_CAM_CY,
                              40u, 1300u);
        assert(!has("$TGT"));
        assert(a.target[0] == tx && a.target[1] == t1);
        puts("PASS: 'A' heartbeat is counted but never solved");
    }

    /* ---------------- 6. STOP 退出闭环 ---------------- */
    send(&a, "$STOP#", 1400u);
    assert(!pipeline_auto_enabled());
    assert(has("$ACK,STOP#"));
    puts("PASS: STOP leaves the closed loop");

    /* ---------------- 7. 手动 $TARGET 不受 AUTO 限制 ---------------- */
    {
        static const uint16_t folded[ARM_SERVO_COUNT] = {900u, 600u, 900u, 0u, 900u, 900u};
        set_servo(&a, folded);
        clean_log();
        send(&a, "$TARGET,80,60,40#", 1500u);
        assert(has("$TGT,OK"));
        assert(!pipeline_auto_enabled());
        puts("PASS: manual $TARGET solves even with AUTO off");
    }

    /* ---------------- 8. $KIN 状态行 ---------------- */
    clean_log();
    send(&a, "$KIN#", 1600u);
    assert(has("$KIN,"));
    puts("PASS: $KIN reports pipeline counters");

    puts("ALL PIPELINE TESTS PASS (not a physical-board test)");
    return 0;
}
