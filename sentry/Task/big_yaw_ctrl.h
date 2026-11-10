/**
  * @file   big_yaw_ctrl.h
  * @brief  大 yaw：MIT 阻抗内环 + ESO 扰动补偿
  */

#ifndef __BIG_YAW_CTRL_H__
#define __BIG_YAW_CTRL_H__

#include "dm_motor.h"

/* 先用估值；测完 J 后改 BIG_YAW_J */
#define BIG_YAW_J          (0.045f)   /* kg·m²，电机侧等效惯量 */
#define BIG_YAW_WC         (8.0f)   /* 闭环带宽 rad/s */
#define BIG_YAW_ZETA       (0.45f)    /* 阻尼比 */
#define BIG_YAW_WLP        (10.0f)   /* 速度反馈低通 Hz */
#define BIG_YAW_NOTCH_F    (19.3f)   /* Hz，机械共振频率，0 = 关陷波 */
#define BIG_YAW_NOTCH_Q    (4.0f)    /* 陷波品质因数，越大越窄 */
#define BIG_YAW_ACC_FF     (0.0f)    /* 加速度前馈增益 [0,1] */
#define BIG_YAW_ACC_LPF    (20.0f)   /* 参考加速度低通 Hz */
#define BIG_YAW_ESO_WO     (60.0f)   /* ESO 带宽 rad/s */
#define BIG_YAW_ESO_GAIN   (0.0f)    /* 先 0 只观测；确认 z2 后改 1 */
#define BIG_YAW_COMP_LPF   (6.0f)    /* 补偿量低通 Hz */
#define BIG_YAW_DLIM       (500.0f)  /* z2 限幅 rad/s² */
#define BIG_YAW_TFF_LIM    (6.0f)    /* t_ff 限幅 N·m，需大于 tau_limit */
#define BIG_YAW_TAU_LIM    (6.0f)    /* 总力矩保护 N·m */
#define BIG_YAW_KI         (1.0f)    /* 积分增益 N·m/(rad·s)，只在目标停住时生效 */
#define BIG_YAW_ILIM       (1.0f)    /* 积分限幅 N·m */
#define BIG_YAW_I_VEL      (0.05f)   /* rad/s，参考速度大于此值时不积分 */
#define BIG_YAW_I_DEADBAND (0.02f)   /* rad，误差小于此值停止积分 */
#define BIG_YAW_I_ERR_MAX  (0.8f)    /* rad，误差大于此值时清零积分 */
#define BIG_YAW_TEST_LIM   (1.5f)    /* 标定力矩限幅 N·m */

typedef struct
{
    float J;            /* 等效惯量 kg·m² */
    float b0;           /* 1/J */
    float wc;           /* 带宽 rad/s */
    float zeta;         /* 阻尼比 */

    float eso_bw;       /* ESO 带宽 rad/s，0 = 关 */
    float eso_comp;     /* 补偿比例 [0,1]，0 = 只观测 */
    float comp_lpf_hz;  /* 补偿量低通 Hz */
    float dist_limit;   /* z2 限幅 rad/s² */

    float tff_limit;    /* t_ff 限幅 N·m */
    float tau_limit;    /* 总力矩限幅 N·m */
    float rate_lpf_hz;  /* 速度反馈低通 Hz */
    float vel_lpf;      /* 低通后的速度 rad/s */
    float nb0;          /* 陷波系数 */
    float nb1;
    float nb2;
    float na1;
    float na2;
    float nx1;          /* 陷波历史：输入 */
    float nx2;
    float ny1;          /* 陷波历史：输出 */
    float ny2;
    uint8_t notch_ready;/* 0 = 系数还没算 */
    float kp_eff;       /* MCU 侧位置刚度 N·m/rad */
    float kd_eff;       /* MCU 侧阻尼 N·m/(rad/s) */

    float acc_ff_gain;  /* 加速度前馈增益 [0,1] */
    float acc_lpf_hz;   /* 参考加速度低通 Hz */
    float vel_ref_last; /* 上一拍参考速度 rad/s */
    float acc_ref_lpf;  /* 低通后的参考加速度 rad/s² */

    float ki;           /* 积分增益 N·m/(rad·s) */
    float i_limit;      /* 积分限幅 N·m */
    float integral;     /* 积分累加 N·m */

    float z1;           /* ESO 速度估计 rad/s */
    float z2;           /* ESO 扰动估计 rad/s² */
    float comp_lpf;     /* 低通后的 z2 */
    float tau_last;     /* 上一拍总力矩 N·m */
    uint8_t started;    /* 0 = 第一拍 */

    float test_torque;  /* 标定：恒定力矩 N·m，0 = 关 */
    float test_time_ms; /* 标定：剩余时间 ms，先写正数再写力矩 */
} big_yaw_ctrl_t;

void big_yaw_ctrl_init(big_yaw_ctrl_t *c);
void big_yaw_ctrl_reset(big_yaw_ctrl_t *c);

void big_yaw_ctrl_step(big_yaw_ctrl_t *c,
                       float pos_ref, float vel_ref,
                       float pos, float vel, float tau_meas,
                       float dt,
                       float *pos_cmd, float *vel_cmd,
                       float *kp, float *kd, float *t_ff);

#endif
