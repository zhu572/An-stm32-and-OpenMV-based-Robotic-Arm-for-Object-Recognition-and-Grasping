#include "handeye.h"
#include <string.h>

/* 默认手眼标定 [TEMP]：纯同轴安装，相机在末端前方 30 mm。
 * 真实安装姿态与偏移必须用标定结果替换。
 * 若相机有横向偏移（例如 (10,20,30)），光轴会偏离臂平面，
 * arm_ik_pose() 会返回约 13° 的接近方向残差，这是 4R 臂的几何限制。 */
static const Mat4 g_default_calib = {
    {
        { 1.0f, 0.0f, 0.0f,  0.0f },
        { 0.0f, 1.0f, 0.0f,  0.0f },
        { 0.0f, 0.0f, 1.0f, 30.0f },
        { 0.0f, 0.0f, 0.0f,  1.0f }
    }
};

static Mat4 g_calib;
static int g_calib_loaded = 0;

void handeye_set_calib(const Mat4 *T_ee_cam)
{
    if (!T_ee_cam) return;
    g_calib = *T_ee_cam;
    g_calib_loaded = 1;
}

const Mat4 *handeye_get_calib(void)
{
    return g_calib_loaded ? &g_calib : &g_default_calib;
}

void pixel_to_cam_point(const CamIntrinsics *k,
                        float u, float v, float depth_mm,
                        float *xc, float *yc, float *zc)
{
    *xc = (u - k->cx) * depth_mm / k->fx;
    *yc = (v - k->cy) * depth_mm / k->fy;
    *zc = depth_mm;
}

void cam_pose_to_base(const Mat4 *T_base_ee,
                      const Mat4 *T_ee_cam,
                      const Mat4 *T_cam_obj,
                      Mat4 *T_base_obj)
{
    Mat4 T_base_cam;

    mat4_mul(T_base_ee, T_ee_cam, &T_base_cam);
    mat4_mul(&T_base_cam, T_cam_obj, T_base_obj);
}

int handeye_pixel_to_base(const CamIntrinsics *k,
                          const Mat4 *T_ee_cam,
                          const Mat4 *T_base_ee,
                          float u, float v, float depth_mm,
                          Pose *out_base_pose)
{
    float xc, yc, zc;
    Mat4 T_cam_obj, T_base_obj;

    if (!T_ee_cam) T_ee_cam = handeye_get_calib();
    if (!k || !T_ee_cam || !T_base_ee || !out_base_pose) return -1;

    pixel_to_cam_point(k, u, v, depth_mm, &xc, &yc, &zc);

    mat4_identity(&T_cam_obj);
    T_cam_obj.m[0][3] = xc;
    T_cam_obj.m[1][3] = yc;
    T_cam_obj.m[2][3] = zc;

    cam_pose_to_base(T_base_ee, T_ee_cam, &T_cam_obj, &T_base_obj);
    mat4_to_pose(&T_base_obj, out_base_pose);
    return 0;
}
