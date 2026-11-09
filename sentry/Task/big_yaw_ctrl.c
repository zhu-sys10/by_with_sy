/**
  * @file   big_yaw_ctrl.c
  * @brief  大 yaw：MIT 阻抗内环 + ESO 扰动补偿
  *
  * 内环仍由 DM4310 MIT 执行：
  *   tau = kp(p_ref - p) + kd(v_ref - v) + t_ff
  * 本模块只算 kp/kd 和 ESO 补偿 t_ff。
  */

#include "big_yaw_ctrl.h"
#include "arm_math.h"

#define TWO_PI_F (6.2831853f)

/* 二阶陷波（RBJ），用来压机械共振，不影响低频段相位 */
static void notch_init(big_yaw_ctrl_t *c, float f0, float q, float dt)
{
    float w0;
    float alpha;
    float a0;

    if ((f0 <= 0.0f) || (q <= 0.0f))
    {
        c->nb0 = 1.0f;      /* 旁路 */
        c->nb1 = 0.0f;
        c->nb2 = 0.0f;
        c->na1 = 0.0f;
        c->na2 = 0.0f;
        return;
    }

    w0    = TWO_PI_F * f0 * dt;
    alpha = arm_sin_f32(w0) / (2.0f * q);
    a0    = 1.0f + alpha;

    c->nb0 =  1.0f / a0;
    c->nb1 = -2.0f * arm_cos_f32(w0) / a0;
    c->nb2 =  1.0f / a0;
    c->na1 = -2.0f * arm_cos_f32(w0) / a0;
    c->na2 = (1.0f - alpha) / a0;
}

static float notch_step(big_yaw_ctrl_t *c, float x)
{
    float y = c->nb0 * x + c->nb1 * c->nx1 + c->nb2 * c->nx2
            - c->na1 * c->ny1 - c->na2 * c->ny2;

    c->nx2 = c->nx1;
    c->nx1 = x;
    c->ny2 = c->ny1;
    c->ny1 = y;

    return y;
}

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

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

/* 一阶低通系数 a = w·dt / (1 + w·dt) */
static float lp_alpha(float fc, float dt)
{
    float w = TWO_PI_F * fc * dt;

    return w / (1.0f + w);
}

void big_yaw_ctrl_reset(big_yaw_ctrl_t *c)
{
    c->z1        = 0.0f;
    c->z2        = 0.0f;
    c->comp_lpf  = 0.0f;
    c->tau_last  = 0.0f;
    c->integral  = 0.0f;
    c->vel_lpf      = 0.0f;
    c->nx1          = 0.0f;
    c->nx2          = 0.0f;
    c->ny1          = 0.0f;
    c->ny2          = 0.0f;
    c->notch_ready  = 0;
    c->kp_eff       = 0.0f;
    c->kd_eff       = 0.0f;
    c->vel_ref_last = 0.0f;
    c->acc_ref_lpf  = 0.0f;
    c->started   = 0;
    c->test_torque  = 0.0f;
    c->test_time_ms = 0.0f;
}

void big_yaw_ctrl_init(big_yaw_ctrl_t *c)
{
    c->J           = BIG_YAW_J;
    c->b0          = 1.0f / BIG_YAW_J;
    c->wc          = BIG_YAW_WC;
    c->zeta        = BIG_YAW_ZETA;
    c->eso_bw      = BIG_YAW_ESO_WO;
    c->eso_comp    = BIG_YAW_ESO_GAIN;
    c->comp_lpf_hz = BIG_YAW_COMP_LPF;
    c->dist_limit  = BIG_YAW_DLIM;
    c->tff_limit   = BIG_YAW_TFF_LIM;
    c->tau_limit   = BIG_YAW_TAU_LIM;
    c->ki          = BIG_YAW_KI;
    c->i_limit     = BIG_YAW_ILIM;
    c->rate_lpf_hz = BIG_YAW_WLP;
    c->acc_ff_gain = BIG_YAW_ACC_FF;
    c->acc_lpf_hz  = BIG_YAW_ACC_LPF;

    big_yaw_ctrl_reset(c);
}

void big_yaw_ctrl_step(big_yaw_ctrl_t *c,
                       float pos_ref, float vel_ref,
                       float pos, float vel, float tau_meas,
                       float dt,
                       float *pos_cmd, float *vel_cmd,
                       float *kp, float *kd, float *t_ff)
{
    float kp_f;
    float kd_f;
    float vel_fb;
    float a_ref;
    float err;
    float comp;
    float tau_total;

    /* 标定模式：kp=kd=0，只给恒定力矩；时间到自动关 */
    if (c->test_torque != 0.0f && c->test_time_ms > 0.0f)
    {
        float tq = clampf(c->test_torque, -BIG_YAW_TEST_LIM, BIG_YAW_TEST_LIM);

        c->test_time_ms -= dt * 1000.0f;
        if (c->test_time_ms <= 0.0f)
        {
            c->test_time_ms = 0.0f;
            c->test_torque  = 0.0f;
            c->started      = 0;    /* 退出后 ESO 重新初始化 */
        }

        *pos_cmd = pos;
        *vel_cmd = 0.0f;
        *kp      = 0.0f;
        *kd      = 0.0f;
        *t_ff    = tq;
        c->tau_last = tau_meas;
        return;
    }

    /* 全外部 PD：电机只出力矩，kp/kd 都在 MCU 算 */
    c->b0 = 1.0f / c->J;
    kp_f  = c->J * c->wc * c->wc;
    kd_f  = 2.0f * c->zeta * c->wc * c->J;

    c->kp_eff = kp_f;
    c->kd_eff = kd_f;

    if (c->rate_lpf_hz > 0.0f)
    {
        c->vel_lpf += lp_alpha(c->rate_lpf_hz, dt) * (vel - c->vel_lpf);
    }
    else
    {
        c->vel_lpf = vel;
    }

    /* 陷波：共振频段增益归零，避免阻尼项在共振频率上变成负阻尼 */
    if (c->notch_ready == 0)
    {
        notch_init(c, BIG_YAW_NOTCH_F, BIG_YAW_NOTCH_Q, dt);
        c->notch_ready = 1;
    }
    vel_fb = notch_step(c, c->vel_lpf);

    /* 第一拍用当前速度初始化 ESO，避免起步冲击 */
    if (c->started == 0)
    {
        c->z1           = vel_fb;
        c->z2           = 0.0f;
        c->comp_lpf     = 0.0f;
        c->tau_last     = 0.0f;
        c->vel_ref_last = vel_ref;
        c->acc_ref_lpf  = 0.0f;
        c->started      = 1;
    }

    /* 参考加速度前馈：起步/刹车提前出力 */
    a_ref = (vel_ref - c->vel_ref_last) / dt;
    c->vel_ref_last = vel_ref;

    if (c->acc_lpf_hz > 0.0f)
    {
        c->acc_ref_lpf += lp_alpha(c->acc_lpf_hz, dt) * (a_ref - c->acc_ref_lpf);
    }
    else
    {
        c->acc_ref_lpf = a_ref;
    }

    /* ESO：z1 → 速度，z2 → 总扰动；输入用实测力矩 */
    if (c->eso_bw > 0.0f)
    {
        float eso_err = c->z1 - vel_fb;
        float b1      = 2.0f * c->eso_bw;
        float b2      = c->eso_bw * c->eso_bw;

        c->z1 += (c->z2 + c->b0 * tau_meas - b1 * eso_err) * dt;
        c->z2 += (-b2 * eso_err) * dt;
        c->z2  = clampf(c->z2, -c->dist_limit, c->dist_limit);
    }
    else
    {
        c->z1 = vel_fb;
        c->z2 = 0.0f;
    }

    /* 只滤补偿量，不滤观测器本身 */
    if (c->eso_bw > 0.0f)
    {
        if (c->comp_lpf_hz > 0.0f)
        {
            c->comp_lpf += lp_alpha(c->comp_lpf_hz, dt) * (c->z2 - c->comp_lpf);
        }
        else
        {
            c->comp_lpf = c->z2;
        }
    }
    else
    {
        c->comp_lpf = 0.0f;
    }

    /* 积分：只在目标停住时慢慢消静差，目标一动就清零 */
    err = pos_ref - pos;

    if ((absf(vel_ref) > BIG_YAW_I_VEL) || (absf(err) > BIG_YAW_I_ERR_MAX))
    {
        c->integral = 0.0f;
    }
    else if (absf(err) > BIG_YAW_I_DEADBAND)
    {
        c->integral = clampf(c->integral + c->ki * err * dt,
                             -c->i_limit, c->i_limit);
    }

    /* 全外部力矩：位置 + 阻尼 + ESO 补偿 + 积分 */
    tau_total = kp_f * err
              + kd_f * (vel_ref - vel_fb)
              + c->J * c->acc_ref_lpf * c->acc_ff_gain
              - c->comp_lpf / c->b0 * c->eso_comp
              + c->integral;
    tau_total = clampf(tau_total, -c->tau_limit, c->tau_limit);
    comp      = tau_total;

    c->tau_last = tau_meas;

    *pos_cmd = pos_ref;
    *vel_cmd = vel_ref;
    *kp      = 0.0f;      /* 电机只出力矩 */
    *kd      = 0.0f;
    *t_ff    = comp;
}
