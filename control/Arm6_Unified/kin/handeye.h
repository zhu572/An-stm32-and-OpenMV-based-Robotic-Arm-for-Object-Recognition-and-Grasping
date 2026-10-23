#ifndef HANDEYE_H
#define HANDEYE_H

#include "mat4.h"

typedef struct {
    float fx, fy;
    float cx, cy;
} CamIntrinsics;

/* 安装手眼标定 T_ee_cam（4x4，EE 系下描述相机系）。
 * 固件无文件系统，标定值由代码里的默认矩阵（handeye.c 中 g_default_calib）
 * 或 $CALIB 命令（后续可扩展）提供。 */
void handeye_set_calib(const Mat4 *T_ee_cam);

/* 当前安装的标定，未设置时返回内置默认矩阵（永不为 NULL）。 */
const Mat4 *handeye_get_calib(void);

/* 像素 + 深度反投影到相机系：
 *   Xc = (u - cx) * d / fx,  Yc = (v - cy) * d / fy,  Zc = d */
void pixel_to_cam_point(const CamIntrinsics *k,
                        float u, float v, float depth_mm,
                        float *xc, float *yc, float *zc);

/* 完整眼在手上链路：
 *   T_base_obj = T_base_ee * T_ee_cam * T_cam_obj */
void cam_pose_to_base(const Mat4 *T_base_ee,
                      const Mat4 *T_ee_cam,
                      const Mat4 *T_cam_obj,
                      Mat4 *T_base_obj);

/* 便捷封装：OpenMV (u, v, depth) → 基座系目标位姿。
 * T_ee_cam 传 NULL 时使用已安装标定。返回 0 成功。 */
int handeye_pixel_to_base(const CamIntrinsics *k,
                          const Mat4 *T_ee_cam,
                          const Mat4 *T_base_ee,
                          float u, float v, float depth_mm,
                          Pose *out_base_pose);

#endif /* HANDEYE_H */
