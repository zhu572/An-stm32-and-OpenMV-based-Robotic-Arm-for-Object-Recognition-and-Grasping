#ifndef VISION_FRAME_H
#define VISION_FRAME_H
#include <stdbool.h>
#include <stdint.h>

/* OpenMV 视觉帧（与 view/main.py 一致）：
 *
 *     0x2C 0x12 <color> <cx> <cy> [depth] 0x5B
 *
 *   color : 'A'..'Z'，view/main.py 用 B/Y/G/R，无目标时为 'A'
 *   cx,cy : 像素坐标，单字节（QQVGA 160x120 足够）
 *   depth : 可选，单字节毫米；缺省时用 ARM_DEFAULT_DEPTH_MM
 *   tail  : 0x5B
 *
 * 解析规则（无需改 view/main.py 即可工作）：
 *   - 载荷满 3 字节后遇到 0x5B 即结束 → 6 字节帧，depth 取默认值
 *   - 载荷满 4 字节后遇到 0x5B 即结束 → 7 字节帧，depth 取第 4 字节
 *   - 载荷不足 3 字节时出现的 0x5B 当作普通数据（像素值可能就是 0x5B）
 *
 * 这是尽力而为的无校验协议：没有 CRC，一个合法字节被打扰成另一个合法
 * 字节无法检测。协议变更需同时改 view/main.py。
 */

#define VISION_HEAD0      0x2Cu
#define VISION_HEAD1      0x12u
#define VISION_TAIL       0x5Bu
#define VISION_TIMEOUT_MS 150u
#define VISION_PAYLOAD_MAX 4u

/* 无目标心跳色：'A'。固件只计帧数、不解算，避免对幻影目标发动作。 */
#define VISION_COLOR_NONE 0x41u

typedef struct {
    uint8_t color;
    uint8_t u, v;
    uint16_t depth;
    uint8_t payload_len;   /* 3 = 无 depth，4 = 带 depth */
} vision_detection_t;

typedef struct {
    uint8_t state;         /* 0 等帧头, 1 等 0x12, 2 收载荷 */
    uint8_t len;
    uint8_t payload[VISION_PAYLOAD_MAX];
    uint32_t last_ms;
    uint16_t frames;       /* 成功组帧数 */
    uint16_t drops;        /* 帧内异常/超时丢弃数 */
} vision_parser_t;

void vision_init(vision_parser_t *p);

/* 逐字节喂入；组帧完成时返回 true 并填写 out。 */
bool vision_feed(vision_parser_t *p, uint8_t byte, uint32_t now,
                 uint16_t default_depth, vision_detection_t *out);

/* 周期调用以回收超时半帧。 */
void vision_poll(vision_parser_t *p, uint32_t now);

#endif /* VISION_FRAME_H */
