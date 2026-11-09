/**
  * @file   gimbal_6020.h
  * @brief  大疆 GM6020 驱动 + 单轴状态反馈 / ESO 控制器
  * @note   模型推导见 小yaw控制器推导.html
  */

#ifndef __GIMBAL_6020_H__
#define __GIMBAL_6020_H__

#include "can.h"

#define GIMBAL_6020_TX_ID   (0x1FEu)    /* 电流控制帧，需先开电流环开关 */
#define GIMBAL_6020_RX_ID   (0x205u)    /* 反馈帧 */
#define GIMBAL_6020_TX_SLOT (0)         /* 本电机在帧里的起始字节：ID1→0 / ID2→2 / ID3→4 / ID4→6 */

#define GIMBAL_6020_OFFSET_ENABLE (1)           /* 0 = 上电首帧锁存零点 */
#define GIMBAL_6020_OFFSET_ECD    (7994u)       /* 机械零位 ecd（0~8191）*/

/* 控制器参数。b0 随负载变，换负载必须重测；其余不用动。
 * 刚性 = b0·kp = wc²，所以"手感硬不硬"只由 wc 决定。
 * ESO 的 wo 往上抬会激起高频共振（comp=0 时不出现），每次提高都要重做压力测试。 */
#define GIMBAL_6020_B0       (0.0095f)  /* counts → rad/s²，带载、大 yaw 固定、±1000 方波实测 */
#define GIMBAL_6020_WC       (10.0f)    /* 带宽 rad/s，稳定测试值 */
#define GIMBAL_6020_ZETA     (0.5f)     /* 阻尼比 */
#define GIMBAL_6020_KI_INT   (0.0f)     /* 积分增益，0 = 关（ESO 已消静差）*/
#define GIMBAL_6020_ILIM     (3000.0f)  /* 积分限幅 counts */
#define GIMBAL_6020_ESO_WO   (80.0f)    /* ESO 带宽 rad/s，0 = 关 */
#define GIMBAL_6020_ESO_GAIN (0.0f)     /* 先 0 只观测；确认 z2 正常后改 1 */
#define GIMBAL_6020_COMP_LPF (6.0f)     /* 补偿量低通 Hz，0 = 不滤 */
#define GIMBAL_6020_DLIM     (500.0f)   /* z2 限幅 rad/s²，需 ≥ b0·|out_last| 峰值 */
#define GIMBAL_6020_CLIM     (16000.0f) /* 输出限幅 counts */
#define GIMBAL_6020_CSLEW    (0.0f)     /* 输出斜率限幅 counts/s，0 = 关 */
#define GIMBAL_6020_TSQ_PERIOD (300.0f) /* 标定方波周期 ms */
#define GIMBAL_6020_TSQ_LIM  (6000.0f)  /* 标定方波限幅 counts */
#define GIMBAL_6020_WLP      (35.0f)    /* 速度反馈低通 Hz，0 = 关 */
#define GIMBAL_6020_WREF_LP  (40.0f)    /* 前馈低通 Hz，0 = 关 */
#define GIMBAL_6020_WREF_LIM (6.0f)     /* 前馈限幅 rad/s */
#define GIMBAL_6020_REF_SLEW (6.0f)     /* 参考限速 rad/s，防大跨距饱和 */

/* 负载耦合前馈：大 yaw → 小 yaw
 * a_couple = −φ̈_b(1 + k·cos(θ_s+α)) − k·φ̇_b²·sin(θ_s+α)
 * u_ff 只加在物理输出上，不进 ESO 模型。 */
#define GIMBAL_6020_FF_ENABLE   (0)
#define GIMBAL_6020_FF_SIGN     (1.0f)      /* 实测方向反了就填 -1 */
#define GIMBAL_6020_KT_EFF      (0.0001355f)/* N·m/count = 0.74×3/16384 */
#define GIMBAL_6020_LOAD_MASS   (0.60f)     /* kg，称重后改 */
#define GIMBAL_6020_LOAD_R      (0.125f)    /* m，质心到小 yaw 轴 */
#define GIMBAL_6020_AXIS_D      (0.092f)    /* m，两轴间距 */
#define GIMBAL_6020_LOAD_ALPHA  (0.0f)      /* rad，质心方位，待辨识 */
#define GIMBAL_6020_FF_ACC_LPF  (30.0f)     /* Hz，大 yaw 角加速度低通 */

/* 方向标定：填错会跑飞或抖震，改完手转验证 */
#define GIMBAL_6020_OUT_TURN  (1.0f)    /* 填反 = 位置环正反馈 */
#define GIMBAL_6020_GYRO_TURN (1.0f)    /* 填反 = 阻尼变负 */

/* ---------------------------- 调参区 ---------------------------- */
typedef struct
{
    float b0;               /* counts → rad/s² */
    float wc;               /* 带宽 rad/s */
    float zeta;             /* 阻尼比 */
    float rate_lpf_hz;      /* 速度反馈低通 Hz，0 = 关 */

    float ki;               /* 积分增益 counts/(rad·s) */
    float i_limit;          /* 积分限幅 counts */

    float eso_bw;           /* ESO 带宽 rad/s，0 = 关 */
    float eso_comp;         /* 补偿比例 [0,1] */
    float comp_lpf_hz;      /* 补偿量低通 Hz，0 = 不滤 */
    float dist_limit;       /* 扰动估计限幅 rad/s² */

    float out_limit;        /* 输出限幅 counts */
    float slew_limit;       /* 输出斜率限幅 counts/s，0 = 关 */

    float ref_lpf_hz;       /* 参考速度低通 Hz */
    float ref_slew;         /* 参考限速 rad/s，0 = 关 */
} yaw_axis_cfg_t;

/* ------------------------ 负载耦合前馈 ------------------------- */
typedef struct
{
    uint8_t enable;         /* 1 = 开，0 = 关 */
    float   sign;           /* +1 / −1 */
    float   mass;           /* kg */
    float   r;              /* m，质心到小 yaw 轴 */
    float   d;              /* m，两轴间距 */
    float   alpha;          /* rad，质心方位 */
    float   out;            /* 当前前馈 counts，只读 */
} yaw_ff_t;

/* -------------------------- 标定 ------------------------------ */
typedef struct
{
    float test_current;     /* 固定电流 counts */
    float test_sq_amp;      /* 方波幅值 counts，0 = 关 */
    float test_sq_period;   /* 方波周期 ms */
    float test_sq_t;        /* 方波计时 ms */
} yaw_calib_t;

/* ------------------------ 运行时状态 --------------------------- */
typedef struct
{
    float kp;               /* wc²/b0 */
    float kd;               /* 2·zeta·wc/b0 */

    float eso_rate;         /* ESO 角速度估计 rad/s */
    float eso_dist;         /* ESO 扰动估计 rad/s² */
    float comp_lpf;         /* 低通后的补偿量 rad/s² */
    float integral;         /* 积分累加 counts */

    float out_last;         /* 上一拍反馈电流，给 ESO 用 */
    float out_last_total;   /* 上一拍物理输出，给斜坡限幅用 */

    float big_rate;         /* 大 yaw 角速度 rad/s */
    float big_accel;        /* 大 yaw 角加速度 rad/s² */

    float rate_lpf;         /* 低通后的反馈角速度 rad/s */
    float rate_last;        /* 原始反馈角速度 rad/s（标定看这个） */

    float ref_last;         /* 上一拍目标角 */
    float ref_shaped;       /* 限速后的参考角 rad */
    float ref_rate_lpf;     /* 低通后的参考角速度 rad/s */
    uint8_t started;        /* 0 = 第一拍还没跑 */
} yaw_axis_state_t;

typedef struct
{
    yaw_axis_cfg_t   cfg;
    yaw_ff_t         ff;
    yaw_calib_t      cal;
    yaw_axis_state_t st;
} axis_ctrl_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;

    uint16_t ecd;           /* 0~8191，原始编码器 */
    int16_t  given_current; /* 反馈帧里的实际电流 counts */
    uint8_t  online;        /* 1 = 收到过反馈 */

    uint16_t offset_ecd;    /* 角度零点 */
    float    angle;         /* rad，相对零点，±π */
    axis_ctrl_t ctrl;
} gimbal_6020_t;

void gimbal_6020_init(gimbal_6020_t *motor, CAN_HandleTypeDef *hcan);
void gimbal_6020_reset(gimbal_6020_t *motor);   /* 清控制器状态，停机时调 */
uint8_t gimbal_6020_decode(gimbal_6020_t *motor, uint32_t std_id, const uint8_t *data);
void gimbal_6020_set_current(gimbal_6020_t *motor, int16_t current);
void gimbal_6020_set_angle(gimbal_6020_t *motor, float angle_set, float y_rate);

#endif
