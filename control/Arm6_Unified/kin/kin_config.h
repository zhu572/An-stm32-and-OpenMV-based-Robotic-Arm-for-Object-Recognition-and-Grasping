#ifndef KIN_CONFIG_H
#define KIN_CONFIG_H

/* ============================================================================
 * kin_config.h - 运动学 / 相机 / 求解器的固定参数（唯一来源）
 *
 * 本文件是 PC 端 control/ds/c语言/arm_config.h 的 MCU 移植版本，
 * 为保证 STM32F103C8（无 FPU）可用，全部改为 float。
 *
 * 单位约定：
 *   长度 mm，关节角 deg（对外），内部弧度。
 *
 * 标注 [TEMP] 的是占位值，必须用实测/标定结果替换：
 *   连杆尺寸、关节限位、相机内参、手眼矩阵。
 * ============================================================================ */

#define KIN_PI 3.14159265358979323846f
#define KIN_TWO_PI (2.0f * KIN_PI)

/* ---------------------------------------------------------------- */
/* 连杆参数  [TEMP] 沿用原型值，待实测                               */
/* 肩关节轴线位于基座系 (0, -KIN_P, KIN_A1)                          */
/* ---------------------------------------------------------------- */
#define KIN_P    140.0f
#define KIN_A1   120.0f
#define KIN_A2   120.0f
#define KIN_A3   120.0f
#define KIN_A4   120.0f

/* ---------------------------------------------------------------- */
/* 关节限位 deg  [TEMP]                                              */
/* j1 基座偏航 / j2 大臂俯仰 / j3 小臂俯仰 / j4 腕部俯仰             */
/*                                                                  */
/* ⚠ 这些限位必须能被 g_joint_map 映射进对应舵机的 0..180° 区间，     */
/*   否则 arm_init() 会直接回 $ERR,CONFIG# 拒绝启动。                */
/*   当前组合（见 arm_config.c）：j1 零位 90° → ±90°；               */
/*   j2/j3 零位 0° → 0..180°；j4 零位 90° → ±90°。                   */
/* ---------------------------------------------------------------- */
#define KIN_J1_MIN ( -90.0f)
#define KIN_J1_MAX (  90.0f)
#define KIN_J2_MIN (   0.0f)
#define KIN_J2_MAX ( 180.0f)
#define KIN_J3_MIN (   0.0f)
#define KIN_J3_MAX ( 180.0f)
#define KIN_J4_MIN ( -90.0f)
#define KIN_J4_MAX (  90.0f)

#define KIN_JOINT_COUNT 4

/* ---------------------------------------------------------------- */
/* 初始 / 参考位姿  [TEMP]                                           */
/* 仅作为 IK 选支的参考锚点，不是机械零位。                          */
/* ---------------------------------------------------------------- */
#define KIN_HOME_J1 (  0.0f)
#define KIN_HOME_J2 ( 45.0f)
#define KIN_HOME_J3 (  0.0f)
#define KIN_HOME_J4 (  0.0f)

/* ---------------------------------------------------------------- */
/* 相机内参  [TEMP]                                                  */
/* view/main.py 用 QQVGA = 160x120，因此主点取画面中心 (80,60)。      */
/* fx/fy 为 QQVGA 下的等效焦距占位值，必须用棋盘格标定替换。          */
/* ---------------------------------------------------------------- */
#define KIN_CAM_W    160.0f
#define KIN_CAM_H    120.0f
#define KIN_CAM_FX   130.0f
#define KIN_CAM_FY   130.0f
#define KIN_CAM_CX    80.0f
#define KIN_CAM_CY    60.0f

/* ---------------------------------------------------------------- */
/* 求解器参数（非物理量）                                            */
/* ---------------------------------------------------------------- */
#define KIN_IK_FAIL      (-999.0f)  /* 失败哨兵值                     */
#define KIN_IK_POSE_TOL  (1.0e-2f)  /* FK 回代位置容差 mm（float）    */
#define KIN_IK_DIR_TOL   (1.0e-3f)  /* 接近方向精确匹配阈值 rad       */
#define KIN_IK_LIMIT_EPS (1.0e-4f)  /* 关节限位比较松弛量 deg         */
#define KIN_IK_COARSE_STEP (5.0f)   /* 仅位置 IK 扫描步长 deg         */
#define KIN_IK_SCAN_MIN  (-90.0f)   /* theta 扫描下界 deg             */
#define KIN_IK_SCAN_MAX  (450.0f)   /* theta 扫描上界 deg             */

#define KIN_D2R(x) ((x) * KIN_PI / 180.0f)
#define KIN_R2D(x) ((x) * 180.0f / KIN_PI)

#endif /* KIN_CONFIG_H */
