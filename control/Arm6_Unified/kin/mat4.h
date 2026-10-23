#ifndef MAT4_H
#define MAT4_H

/* 4x4 齐次矩阵与位姿（位置 + 四元数）工具，float 版。
 * 由 PC 端 control/ds/c语言/mat4.c 移植，去掉 stdio 依赖。 */

typedef struct {
    float m[4][4];
} Mat4;

typedef struct {
    float x, y, z;
    float qw, qx, qy, qz;
} Pose;

void mat4_identity(Mat4 *out);
void mat4_copy(const Mat4 *src, Mat4 *dst);
void mat4_mul(const Mat4 *a, const Mat4 *b, Mat4 *out);
void mat4_from_pose(const Pose *p, Mat4 *out);
void mat4_to_pose(const Mat4 *m, Pose *out);
void quat_normalize(Pose *p);
float pose_trans_error(const Pose *a, const Pose *b);
float pose_rot_error(const Pose *a, const Pose *b);

#endif /* MAT4_H */
