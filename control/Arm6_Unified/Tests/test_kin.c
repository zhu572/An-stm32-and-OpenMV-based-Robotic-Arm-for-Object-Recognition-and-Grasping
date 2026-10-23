#include <stdio.h>
#include <math.h>
#include "mat4.h"
#include "kin_config.h"
#include "arm_fk.h"
#include "arm_ik.h"
#include "handeye.h"

/* 主机侧运动学自检（不参与 Keil 固件编译）。
 * 运行：python Tools/test_host.py
 * 注意：本文件在交付环境中未经编译运行，首次运行如有问题请回传输出。 */

static int g_checks = 0;
static int g_failed = 0;

static void check(int ok, const char *what)
{
    ++g_checks;
    if (!ok) ++g_failed;
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}

static float dist3(float ax, float ay, float az, float bx, float by, float bz)
{
    float dx = ax - bx, dy = ay - by, dz = az - bz;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

int main(void)
{
    CamIntrinsics k;
    Mat4 T_ee;
    int i;

    printf("Arm6_Unified kinematics host test (mm / deg)\n");

    k.fx = KIN_CAM_FX;
    k.fy = KIN_CAM_FY;
    k.cx = KIN_CAM_CX;
    k.cy = KIN_CAM_CY;

    /* ---------------- 1. FK 两个接口一致 ---------------- */
    {
        float j1 = -30.0f, j2 = 40.0f, j3 = 50.0f, j4 = -20.0f;
        float x, y, z;
        arm_fk(j1, j2, j3, j4, &x, &y, &z);
        arm_fk_pose(j1, j2, j3, j4, &T_ee);
        check(dist3(x, y, z, T_ee.m[0][3], T_ee.m[1][3], T_ee.m[2][3]) < 1.0e-3f,
              "arm_fk() and arm_fk_pose() agree");
    }

    /* ---------------- 2. 相机反投影 ---------------- */
    {
        float xc, yc, zc;
        pixel_to_cam_point(&k, KIN_CAM_CX, KIN_CAM_CY, 200.0f, &xc, &yc, &zc);
        check(fabsf(xc) < 1.0e-4f && fabsf(yc) < 1.0e-4f && fabsf(zc - 200.0f) < 1.0e-4f,
              "principal point back-projects onto the optical axis");
    }

    /* ---------------- 3. 手眼链（同轴默认标定） ---------------- */
    {
        Pose p;
        float z_cam;
        arm_fk_pose(0.0f, 90.0f, 0.0f, 0.0f, &T_ee);
        /* 该位姿下 EE z 轴竖直向上，相机在末端前方 30 mm */
        z_cam = T_ee.m[2][3] + 30.0f;
        check(handeye_pixel_to_base(&k, NULL, &T_ee,
                                    KIN_CAM_CX, KIN_CAM_CY, 100.0f, &p) == 0,
              "handeye_pixel_to_base() succeeds with installed calibration");
        check(fabsf(p.z - (z_cam + 100.0f)) < 1.0e-2f &&
              fabsf(p.x - T_ee.m[0][3]) < 1.0e-2f &&
              fabsf(p.y - T_ee.m[1][3]) < 1.0e-2f,
              "center pixel maps to EE axis + depth along tool z");
    }

    /* ---------------- 4. FK / IK 往返 ---------------- */
    {
        static const float cases[][4] = {
            {   0.0f,  45.0f,   0.0f,   0.0f },
            { -30.0f,  40.0f,  50.0f, -20.0f },
            {  30.0f,  60.0f,  30.0f,  10.0f },
            {  90.0f,  20.0f,  60.0f, -30.0f },
            { -90.0f,  80.0f,  20.0f,  20.0f },
            {  15.0f, 100.0f,  40.0f,  30.0f }
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));
        float worst_pos = 0.0f;
        float worst_dir = 0.0f;

        for (i = 0; i < n; ++i) {
            Mat4 T, back;
            float j1, j2, j3, j4, derr = -1.0f;
            float err;

            arm_fk_pose(cases[i][0], cases[i][1], cases[i][2], cases[i][3], &T);
            if (arm_ik_pose(&T, &derr, &j1, &j2, &j3, &j4) != 0) {
                printf("    case %d: IK failed\n", i);
                worst_pos = 1.0e9f;
                continue;
            }
            arm_fk_pose(j1, j2, j3, j4, &back);
            err = dist3(T.m[0][3], T.m[1][3], T.m[2][3],
                        back.m[0][3], back.m[1][3], back.m[2][3]);
            if (err > worst_pos) worst_pos = err;
            if (derr > worst_dir) worst_dir = derr;
            printf("    case %d: pos err %.3e mm  dir err %.3e rad  joints (%.2f %.2f %.2f %.2f)\n",
                   i, err, derr, j1, j2, j3, j4);
        }
        check(worst_pos < 1.0e-2f, "all round trips reproduce the position");
        /* float 下 acos 在 1 附近会被放大，方向容差放宽到 1e-2 rad (~0.57 deg) */
        check(worst_dir < 1.0e-2f, "all round trips reproduce the approach direction");
    }

    /* ---------------- 5. 不可达目标 ---------------- */
    {
        float j1 = 0.0f, j2 = 0.0f, j3 = 0.0f, j4 = 0.0f;
        int rc = arm_ik(0.0f, -KIN_P, 700.0f, &j1, &j2, &j3, &j4);
        check(rc == -1, "unreachable target reports -1");
        check(j1 == KIN_IK_FAIL && j4 == KIN_IK_FAIL,
              "failed solve fills the sentinel");
    }

    printf("%s: %d checks, %d failed\n",
           g_failed == 0 ? "ALL KIN TESTS PASSED" : "KIN TESTS FAILED",
           g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
