#include "protocol.h"
#include <string.h>

static const char *spaces(const char *s) {
    while (*s == ' ' || *s == '\t') ++s;
    return s;
}

/* 不使用 sscanf/浮点/堆：只接受最多一位小数的十进制角度。
 * 拒绝正负号、指数、空字段、NaN、多余字段与超过 180.0 的值。 */
static bool angle(const char **cursor, uint16_t *out) {
    const char *s = spaces(*cursor);
    unsigned whole = 0u, fraction = 0u;
    if (*s < '0' || *s > '9') return false;
    do {
        whole = whole * 10u + (unsigned)(*s++ - '0');
        if (whole > 180u) return false;
    } while (*s >= '0' && *s <= '9');
    if (*s == '.') {
        ++s;
        if (*s < '0' || *s > '9') return false;
        fraction = (unsigned)(*s++ - '0');
    }
    if (whole * 10u + fraction > 1800u) return false;
    *out = (uint16_t)(whole * 10u + fraction);
    *cursor = spaces(s);
    return true;
}

/* 纯十进制无符号整数（用于像素/深度字段），最多 5 位。 */
static bool count(const char **cursor, unsigned max, unsigned *out) {
    const char *s = spaces(*cursor);
    unsigned value = 0u, digits = 0u;
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') {
        value = value * 10u + (unsigned)(*s++ - '0');
        if (++digits > 5u || value > max) return false;
    }
    *out = value;
    *cursor = spaces(s);
    return true;
}

static bool parse(const char *s, arm_command_t *out) {
    static const char *const names[] = {"PING", "STATUS", "ON", "OFF", "STOP", "HOME"};
    for (unsigned k = 0; k < sizeof(names) / sizeof(names[0]); ++k) {
        if (strcmp(s, names[k]) == 0) { out->kind = (command_kind_t)k; return true; }
    }
    if (strcmp(s, "KIN") == 0) { out->kind = CMD_KIN; return true; }
    if (strncmp(s, "AUTO,", 5u) == 0) {
        s += 5;
        if (*s != '0' && *s != '1') return false;
        out->arg[0] = (uint16_t)(*s - '0');
        if (*++s != '\0') return false;
        out->kind = CMD_AUTO;
        return true;
    }
    if (strncmp(s, "TARGET,", 7u) == 0) {
        unsigned u, v, d = ARM_DEFAULT_DEPTH_MM;
        s += 7;
        if (!count(&s, 255u, &u)) return false;
        if (*s++ != ',') return false;
        if (!count(&s, 255u, &v)) return false;
        if (*s == ',') { ++s; if (!count(&s, 65535u, &d)) return false; }
        if (*s != '\0') return false;
        out->arg[0] = (uint16_t)u;
        out->arg[1] = (uint16_t)v;
        out->arg[2] = (uint16_t)d;
        out->kind = CMD_TARGET;
        return true;
    }
    if (s[0] == 'S' && s[1] == ',') {
        s += 2;
        if (*s < '1' || *s > '6') return false;
        out->channel = (uint8_t)(*s++ - '1');
        if (*s++ != ',') return false;
        if (!angle(&s, &out->angles[0]) || *s != '\0') return false;
        out->kind = CMD_SINGLE;
        return true;
    }
    for (unsigned k = 0; k < ARM_SERVO_COUNT; ++k) {
        if (!angle(&s, &out->angles[k])) return false;
        if (k + 1u < ARM_SERVO_COUNT) { if (*s++ != ',') return false; }
    }
    if (*s != '\0') return false;
    out->kind = CMD_MOVE;
    return true;
}

void protocol_reset(protocol_t *p) { memset(p, 0, sizeof(*p)); }

rx_result_t protocol_expire(protocol_t *p, uint32_t now) {
    if (p->collecting && (uint32_t)(now - p->last_byte_ms) >= ARM_FRAME_TIMEOUT_MS) {
        protocol_reset(p);
        return RX_TIMEOUT;
    }
    return RX_NONE;
}

rx_result_t protocol_feed(protocol_t *p, uint8_t byte, uint32_t now, arm_command_t *out) {
    if (byte == '$') {
        p->collecting = true; p->length = 0; p->last_byte_ms = now;
        return RX_NONE;
    }
    if (!p->collecting) return RX_NONE; /* 帧外噪声（含 CR/LF）忽略 */
    p->last_byte_ms = now;
    if (byte == '#') {
        p->frame[p->length] = '\0'; p->collecting = false;
        return parse(p->frame, out) ? RX_COMMAND : RX_FORMAT;
    }
    if (byte < 32u || byte > 126u) {
        protocol_reset(p); return RX_FORMAT;
    }
    if (p->length + 1u >= ARM_FRAME_CAPACITY) {
        protocol_reset(p); return RX_TOO_LONG;
    }
    p->frame[p->length++] = (char)byte;
    return RX_NONE;
}
