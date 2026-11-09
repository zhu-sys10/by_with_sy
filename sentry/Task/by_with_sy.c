/**
  * @file   by_with_sy.c
  * @brief  双 yaw 台架任务：手动 / 世界角瞄准
  *
  * 左拨杆：上 = 世界角瞄准，中 = 手动，下 = 停机
  * 瞄准模式：右摇杆微调世界目标角，左摇杆手动转大 yaw（小 yaw 自动补偿）
  * 从一个会使用控制器的人，逐渐变成一个能够设计控制器的人
  * 测试
  */

#include "by_with_sy.h"
#include "yaw_aim.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "remote_control.h"
#include "INS_task.h"
#include "gimbal_6020.h"
#include "dm_motor.h"
#include "big_yaw_ctrl.h"

dm_motor_t     dm_4310;
gimbal_6020_t  g6020;
big_yaw_ctrl_t big_ctrl;

#define YAW_MODE_SW        (rc_ctrl.rc.s[1])   /* 左拨杆 */
#define YAW_BIG_RC_CH      (2)                 /* 左摇杆左右 */
#define YAW_SMALL_RC_CH    (0)                 /* 右摇杆左右 */
#define RC_CH_MAX          (660.0f)            /* 摇杆满量程 */
#define RC_DEADBAND        (10)
#define YAW_BIG_VEL_WIN    (10)                /* 速度估计窗口 ms，越大越平滑、延迟越大 */
#define YAW_CONTROL_DT     (0.001f)
#define YAW_TWO_PI         (6.2831853f)

/* 摇杆满杆对应的角速度 rad/s（负号 = 旧版方向） */
#define YAW_BIG_MAN_RATE     (-3.0f)    /* 手动模式：大 yaw */
#define YAW_BIG_ACC          (30.0f)    /* 大 yaw 参考角加速度 rad/s² */
#define YAW_SMALL_MAN_RATE   (-3.6f)    /* 手动模式：小 yaw */
#define YAW_AIM_TRIM_RATE    (-3.0f)    /* 瞄准模式：右摇杆微调世界目标 */
#define YAW_AIM_BIG_MAN_RATE (-4.0f)    /* 瞄准模式：左摇杆转大 yaw */

typedef enum
{
    YAW_MODE_DISARM = 0,
    YAW_MODE_MANUAL,
    YAW_MODE_AIM,
} yaw_mode_e;

volatile float yaw_big_set     = 0.0f;   /* 大 yaw 目标 rad（相对固定零点）*/
volatile float yaw_small_set   = 0.0f;   /* 小 yaw 目标 rad（相对大 yaw）*/
volatile float yaw_big_rate    = 0.0f;   /* 大 yaw 参考角速度 rad/s */
static float   s_big_rate_shaped = 0.0f; /* 手动模式限加速度后的参考速度 */
volatile float big_yaw_kp      = 0.0f;   /* 大 yaw MIT kp，只读 */
volatile float big_yaw_kd      = 0.0f;   /* 大 yaw MIT kd，只读 */
volatile float big_yaw_tff     = 0.0f;   /* 大 yaw 补偿力矩 N·m，只读 */
volatile uint8_t big_yaw_free  = 0;      /* 1 = 大 yaw 零力矩，手转标定用 */
volatile float yaw_big_vel     = 0.0f;   /* 由 pos_cont 差分估计的电机角速度 rad/s，只读 */

/* 电机自带 vel 字段噪声 std ≈0.7 rad/s（实测），改用位置差分：窗口 10 ms 时噪声 ≈0.05 */
static float   s_pos_hist[YAW_BIG_VEL_WIN];
static uint8_t s_pos_idx;
static uint8_t s_vel_cnt;

/* 调试观测量（只读）*/
volatile float yaw_big_angle   = 0.0f;   /* 大 yaw 轴角 rad */
volatile float yaw_world_angle = 0.0f;   /* 世界角 rad，以 IMU 为准 */
volatile float yaw_enc_angle   = 0.0f;   /* 编码器推算的世界角，和上面对比用 */
volatile float yaw_world_enc_diff = 0.0f; /* 世界角 − 编码器推算角，应恒定 */

static float clampf(float v, float lo, float hi)
{
    if (v > hi)
    {
        return hi;
    }
    if (v < lo)
    {
        return lo;
    }
    return v;
}

/* 摇杆 → -1.0 ~ +1.0，死区内返回 0 */
static float rc_stick(int16_t ch)
{
    if (ch > RC_DEADBAND || ch < -RC_DEADBAND)
    {
        return (float)ch / RC_CH_MAX;
    }
    return 0.0f;
}

/* 手动模式：摇杆角速度积分成目标角 */
static void yaw_set_update(void)
{
    float rate_cmd = rc_stick(rc_ctrl.rc.ch[YAW_BIG_RC_CH]) * YAW_BIG_MAN_RATE;
    float dmax     = YAW_BIG_ACC * YAW_CONTROL_DT;

    /* 参考角速度限加速度，避免阶跃力矩 */
    s_big_rate_shaped += clampf(rate_cmd - s_big_rate_shaped, -dmax, dmax);
    yaw_big_rate   = s_big_rate_shaped;
    yaw_big_set   += yaw_big_rate * YAW_CONTROL_DT;
    yaw_small_set += rc_stick(rc_ctrl.rc.ch[YAW_SMALL_RC_CH])
                     * YAW_SMALL_MAN_RATE * YAW_CONTROL_DT;

    /* 位置被限幅时参考速度必须归零，否则 kd 前馈会继续把轴推出设定值 */
    if (yaw_big_set > YAW_BIG_MAX_RAD)
    {
        yaw_big_set       = YAW_BIG_MAX_RAD;
        yaw_big_rate      = 0.0f;
        s_big_rate_shaped = 0.0f;
    }
    else if (yaw_big_set < -YAW_BIG_MAX_RAD)
    {
        yaw_big_set       = -YAW_BIG_MAX_RAD;
        yaw_big_rate      = 0.0f;
        s_big_rate_shaped = 0.0f;
    }

    yaw_small_set = clampf(yaw_small_set, -YAW_SMALL_MAX_RAD, YAW_SMALL_MAX_RAD);
}

/* 瞄准模式：右摇杆微调世界目标角 */
static void yaw_target_trim(void)
{
    float stick = rc_stick(rc_ctrl.rc.ch[YAW_SMALL_RC_CH]);

    if (stick != 0.0f)
    {
        yaw_aim_set_target(yaw_aim_get_target()
                           + stick * YAW_AIM_TRIM_RATE * YAW_CONTROL_DT);
    }
}

static yaw_mode_e yaw_mode_read(void)
{
    if (RC_data_is_error() != 0 || switch_is_down(YAW_MODE_SW))
    {
        return YAW_MODE_DISARM;
    }
    if (switch_is_up(YAW_MODE_SW))
    {
        return YAW_MODE_AIM;
    }
    return YAW_MODE_MANUAL;
}

/* ------------------------------ 应用状态 ------------------------------ */

typedef struct
{
    yaw_mode_e mode_last;       /* 上一拍模式 */
    uint8_t    aim_need_init;   /* 已拨到瞄准，等待锁目标 */
    uint8_t    big_latched;     /* 大 yaw 零点已锁存 */
    float      big_start;       /* 大 yaw 固定零点 */
    float      big_rate_last;   /* 大 yaw 角速度，差分用 */
    float      big_accel_lpf;   /* 大 yaw 角加速度，低通后 */
} yaw_app_t;

static yaw_app_t app;

static void yaw_app_init(void)
{
    app.big_start = BIG_START_FIXED;

    MX_USB_DEVICE_Init();

    dm_motor_init(&dm_4310, &hcan1, DM_4310_CAN_ID, DM_4310_MASTER_ID);
    gimbal_6020_init(&g6020, &hcan1);
    big_yaw_ctrl_init(&big_ctrl);

    osDelay(DM_START_DELAY_MS);
}

/* 大 yaw 零点锁存 + 世界角 / 编码器角观测 */
static void yaw_observe(void)
{
    if (dm_4310.online != 0 && app.big_latched == 0)
    {
        app.big_start   = BIG_START_FIXED;
        app.big_latched = 1;
    }

    /* 速度：用位置差分，不用电机自带的 vel 字段（太脏） */
    s_pos_hist[s_pos_idx] = dm_4310.pos_cont;
    s_pos_idx = (uint8_t)((s_pos_idx + 1) % YAW_BIG_VEL_WIN);

    if (s_vel_cnt < YAW_BIG_VEL_WIN)
    {
        s_vel_cnt++;
        yaw_big_vel = 0.0f;
    }
    else
    {
        yaw_big_vel = (dm_4310.pos_cont - s_pos_hist[s_pos_idx])
                    / (YAW_BIG_VEL_WIN * YAW_CONTROL_DT);
    }

    yaw_big_angle      = (dm_4310.pos_cont - app.big_start) * YAW_BIG_RATIO;
    yaw_world_angle    = INS_data.angle_yaw;
    yaw_enc_angle      = yaw_big_angle + g6020.angle;
    yaw_world_enc_diff = yaw_world_angle - yaw_enc_angle;
}

static void yaw_mode_switch(yaw_mode_e mode)
{
    if (mode == app.mode_last)
    {
        return;
    }

    if (mode == YAW_MODE_DISARM)
    {
        gimbal_6020_reset(&g6020);
        big_yaw_ctrl_reset(&big_ctrl);
        yaw_big_rate        = 0.0f;
        app.big_rate_last   = YAW_BIG_RATIO * yaw_big_vel;
        app.big_accel_lpf   = 0.0f;
        s_big_rate_shaped   = 0.0f;
    }
    else if (mode == YAW_MODE_AIM)
    {
        app.aim_need_init = 1;
    }
    else if (mode == YAW_MODE_MANUAL)
    {
        /* 进手动：目标对齐当前位置，避免追旧目标 */
        yaw_big_set       = yaw_big_angle;
        yaw_small_set     = g6020.angle;
        s_big_rate_shaped = 0.0f;
        big_yaw_ctrl_reset(&big_ctrl);
    }

    app.mode_last = mode;
}

/* 大 yaw 角速度/角加速度，给小 yaw 耦合前馈 */
static void yaw_big_dynamics(void)
{
    float big_rate = (dm_4310.online != 0)
                   ? YAW_BIG_RATIO * yaw_big_vel : 0.0f;

    if (dm_4310.online != 0)
    {
        float raw = (big_rate - app.big_rate_last) / YAW_CONTROL_DT;
        float w   = YAW_TWO_PI * GIMBAL_6020_FF_ACC_LPF * YAW_CONTROL_DT;
        float a   = w / (1.0f + w);

        app.big_accel_lpf += a * (raw - app.big_accel_lpf);
    }
    else
    {
        app.big_accel_lpf = 0.0f;
    }

    app.big_rate_last    = big_rate;
    g6020.ctrl.st.big_rate  = big_rate;
    g6020.ctrl.st.big_accel = app.big_accel_lpf;
}

/* 进入瞄准：把当前世界角锁成目标，大 yaw 参考对齐当前位置 */
static void yaw_aim_init_if_needed(void)
{
    if (app.aim_need_init && app.big_latched != 0 && dm_4310.online != 0)
    {
        yaw_aim_enter(INS_data.angle_yaw, yaw_big_angle);
        app.aim_need_init = 0;
    }
}

/* 目标角：瞄准算法或手动积分 */
static void yaw_target_update(yaw_mode_e mode)
{
    yaw_big_rate = 0.0f;

    if (mode == YAW_MODE_AIM && app.aim_need_init == 0)
    {
        float big_cmd;
        float big_rate;
        float small_cmd;
        float big_manual_rate = rc_stick(rc_ctrl.rc.ch[YAW_BIG_RC_CH])
                                * YAW_AIM_BIG_MAN_RATE;

        yaw_target_trim();
        yaw_aim_update(YAW_CONTROL_DT, INS_data.angle_yaw, g6020.angle,
                       yaw_big_angle, big_manual_rate,
                       &big_cmd, &big_rate, &small_cmd);
        yaw_big_set   = big_cmd;
        yaw_big_rate  = big_rate;
        yaw_small_set = small_cmd;
    }
    else if (mode == YAW_MODE_MANUAL)
    {
        yaw_set_update();
    }
}

static void yaw_output_update(yaw_mode_e mode)
{
    /* 大 yaw：位置环在电机内部 */
    if (dm_4310.online != 0)
    {
        if (dm_4310.err != DM_ERR_ENABLED)
        {
            dm_motor_enable(&dm_4310);
        }
        if (DM_ERR_IS_FAULT(dm_4310.err) == 0)
        {
            float pos_cmd;
            float vel_cmd;
            float kp;
            float kd;
            float t_ff;
            float pos_ref;

            /* 标定用：零力矩，可手转读 pos_cont；目标跟随实际，退出不跳 */
            if (big_yaw_free != 0)
            {
                yaw_big_set  = yaw_big_angle;
                yaw_big_rate = 0.0f;
                big_yaw_tff  = 0.0f;
                dm_motor_mit_full(&dm_4310, dm_4310.pos_cont, 0.0f, 0.0f, 0.0f, 0.0f);
            }
            else
            {
                /* 反馈已按 25 rad 展开，目标和反馈在同一个连续域，直接做差 */
                pos_ref = app.big_start + yaw_big_set / YAW_BIG_RATIO;

                big_yaw_ctrl_step(&big_ctrl,
                                  pos_ref,
                                  yaw_big_rate / YAW_BIG_RATIO,
                                  dm_4310.pos_cont, yaw_big_vel, dm_4310.torque,
                                  YAW_CONTROL_DT,
                                  &pos_cmd, &vel_cmd, &kp, &kd, &t_ff);

                big_yaw_kp  = big_ctrl.kp_eff;   /* MCU 侧有效刚度 */
                big_yaw_kd  = big_ctrl.kd_eff;   /* MCU 侧有效阻尼 */
                big_yaw_tff = t_ff;

                dm_motor_mit_full(&dm_4310, pos_cmd, vel_cmd, kp, kd, t_ff);
            }
        }
    }

    /* 小 yaw：相对角速度 = 陀螺仪 − 大 yaw 角速度 */
    if (mode == YAW_MODE_DISARM)
    {
        gimbal_6020_set_current(&g6020, 0);
    }
    else
    {
        float rate_rel = GIMBAL_6020_GYRO_TURN * INS_data.wz
                       - YAW_BIG_RATIO * yaw_big_vel;

        gimbal_6020_set_angle(&g6020, yaw_small_set, rate_rel);
    }
}

void test_task(void const *argument)
{
    yaw_app_init();

    for (;;)
    {
        yaw_mode_e mode = yaw_mode_read();

        yaw_observe();
        yaw_mode_switch(mode);
        yaw_big_dynamics();
        yaw_aim_init_if_needed();


        yaw_target_update(mode);
        yaw_output_update(mode);

        osDelay(1);
    }
}
