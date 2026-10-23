#include "mat4.h"
#include <math.h>
#include <string.h>

void mat4_identity(Mat4 *out)
{
    memset(out, 0, sizeof(*out));
    out->m[0][0] = 1.0f;
    out->m[1][1] = 1.0f;
    out->m[2][2] = 1.0f;
    out->m[3][3] = 1.0f;
}

void mat4_copy(const Mat4 *src, Mat4 *dst)
{
    *dst = *src;
}

void mat4_mul(const Mat4 *a, const Mat4 *b, Mat4 *out)
{
    Mat4 tmp;
    int i, j, k;
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (k = 0; k < 4; ++k) {
                s += a->m[i][k] * b->m[k][j];
            }
            tmp.m[i][j] = s;
        }
    }
    *out = tmp;
}

void quat_normalize(Pose *p)
{
    float n = sqrtf(p->qw * p->qw + p->qx * p->qx + p->qy * p->qy + p->qz * p->qz);
    if (n < 1.0e-12f) {
        p->qw = 1.0f;
        p->qx = 0.0f;
        p->qy = 0.0f;
        p->qz = 0.0f;
        return;
    }
    p->qw /= n;
    p->qx /= n;
    p->qy /= n;
    p->qz /= n;
}

void mat4_from_pose(const Pose *p, Mat4 *out)
{
    float qw = p->qw, qx = p->qx, qy = p->qy, qz = p->qz;
    float n = qw * qw + qx * qx + qy * qy + qz * qz;

    if (n < 1.0e-12f) {
        qw = 1.0f;
        qx = 0.0f;
        qy = 0.0f;
        qz = 0.0f;
    } else {
        float inv = 1.0f / sqrtf(n);
        qw *= inv;
        qx *= inv;
        qy *= inv;
        qz *= inv;
    }

    out->m[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
    out->m[0][1] = 2.0f * (qx * qy - qz * qw);
    out->m[0][2] = 2.0f * (qx * qz + qy * qw);
    out->m[0][3] = p->x;

    out->m[1][0] = 2.0f * (qx * qy + qz * qw);
    out->m[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
    out->m[1][2] = 2.0f * (qy * qz - qx * qw);
    out->m[1][3] = p->y;

    out->m[2][0] = 2.0f * (qx * qz - qy * qw);
    out->m[2][1] = 2.0f * (qy * qz + qx * qw);
    out->m[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
    out->m[2][3] = p->z;

    out->m[3][0] = 0.0f;
    out->m[3][1] = 0.0f;
    out->m[3][2] = 0.0f;
    out->m[3][3] = 1.0f;
}

void mat4_to_pose(const Mat4 *m, Pose *out)
{
    float r00 = m->m[0][0], r11 = m->m[1][1], r22 = m->m[2][2];
    float tr = r00 + r11 + r22;

    out->x = m->m[0][3];
    out->y = m->m[1][3];
    out->z = m->m[2][3];

    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        out->qw = 0.25f * s;
        out->qx = (m->m[2][1] - m->m[1][2]) / s;
        out->qy = (m->m[0][2] - m->m[2][0]) / s;
        out->qz = (m->m[1][0] - m->m[0][1]) / s;
    } else if (r00 > r11 && r00 > r22) {
        float s = sqrtf(1.0f + r00 - r11 - r22) * 2.0f;
        out->qw = (m->m[2][1] - m->m[1][2]) / s;
        out->qx = 0.25f * s;
        out->qy = (m->m[0][1] + m->m[1][0]) / s;
        out->qz = (m->m[0][2] + m->m[2][0]) / s;
    } else if (r11 > r22) {
        float s = sqrtf(1.0f + r11 - r00 - r22) * 2.0f;
        out->qw = (m->m[0][2] - m->m[2][0]) / s;
        out->qx = (m->m[0][1] + m->m[1][0]) / s;
        out->qy = 0.25f * s;
        out->qz = (m->m[1][2] + m->m[2][1]) / s;
    } else {
        float s = sqrtf(1.0f + r22 - r00 - r11) * 2.0f;
        out->qw = (m->m[1][0] - m->m[0][1]) / s;
        out->qx = (m->m[0][2] + m->m[2][0]) / s;
        out->qy = (m->m[1][2] + m->m[2][1]) / s;
        out->qz = 0.25f * s;
    }
    quat_normalize(out);
}

float pose_trans_error(const Pose *a, const Pose *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

float pose_rot_error(const Pose *a, const Pose *b)
{
    float dot = a->qw * b->qw + a->qx * b->qx + a->qy * b->qy + a->qz * b->qz;
    if (dot < 0.0f) dot = -dot;
    if (dot > 1.0f) dot = 1.0f;
    return 2.0f * acosf(dot);
}
