#include "vision_frame.h"
#include <string.h>

enum { VF_IDLE = 0, VF_HEAD1 = 1, VF_PAYLOAD = 2 };

void vision_init(vision_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = VF_IDLE;
}

bool vision_feed(vision_parser_t *p, uint8_t byte, uint32_t now,
                 uint16_t default_depth, vision_detection_t *out)
{
    switch (p->state) {
    case VF_IDLE:
        if (byte == VISION_HEAD0) {
            p->state = VF_HEAD1;
            p->last_ms = now;
        }
        return false;

    case VF_HEAD1:
        p->last_ms = now;
        if (byte == VISION_HEAD1) {
            p->state = VF_PAYLOAD;
            p->len = 0u;
        } else if (byte != VISION_HEAD0) {
            p->state = VF_IDLE;          /* 伪帧头，丢弃 */
        }
        return false;

    default: /* VF_PAYLOAD */
        p->last_ms = now;
        /* 只有载荷满 3 字节后出现的 0x5B 才可能是帧尾：
         * 不足 3 字节时像素值本身就可能等于 0x5B。 */
        if (p->len >= 3u && byte == VISION_TAIL) {
            if (!out) { p->state = VF_IDLE; p->len = 0u; ++p->drops; return false; }
            out->color = p->payload[0];
            out->u = p->payload[1];
            out->v = p->payload[2];
            out->payload_len = p->len;
            out->depth = (p->len >= 4u) ? (uint16_t)p->payload[3] : default_depth;
            p->state = VF_IDLE;
            p->len = 0u;
            ++p->frames;
            return true;
        }
        if (p->len >= VISION_PAYLOAD_MAX) {
            p->state = VF_IDLE;          /* 超长，重新同步 */
            p->len = 0u;
            ++p->drops;
            return false;
        }
        p->payload[p->len++] = byte;
        return false;
    }
}

void vision_poll(vision_parser_t *p, uint32_t now)
{
    if (p->state != VF_IDLE && (uint32_t)(now - p->last_ms) >= VISION_TIMEOUT_MS) {
        p->state = VF_IDLE;
        p->len = 0u;
        ++p->drops;
    }
}
