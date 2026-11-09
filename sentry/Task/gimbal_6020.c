/**
  * @file   gimbal_6020.c
  * @brief  大疆 GM6020 驱动 + 单轴状态反馈 / ESO 控制器
  */

#include "gimbal_6020.h"
#include "arm_math.h"   /* arm_sin_f32 / arm_cos_f32 */

#define GIMBAL_6020_DT  (0.001f)            /* 控制周期 1ms */
#define ECD_PER_TURN    (8191)              /* 编码器一圈刻度数 */
#define ECD_HALF_TURN   (4096)              /* 半圈，过零绕回阈值 */
#define ECD_TO_RAD      (0.0007670844f)     /* 2π / 8191 */
#define PI_F            (3.1415926536f)
#define TWO_PI_F        (6.2831853072f)

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

/* 一阶低通系数 a = w·dt / (1 + w·dt)，w = 2π·fc */
static float lp_alpha(float fc, float dt)
{
    float w = TWO_PI_F * fc * dt;

    return w / (1.0f + w);
}

/* ecd → 相对零点角度 rad，范围 ±π */
static float ecd_to_angle(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t rel = (int32_t)ecd - (int32_t)offset_ecd;

    if (rel > ECD_HALF_TURN)
    {
        rel -= ECD_PER_TURN;
    }
    else if (rel < -ECD_HALF_TURN)
    {
        rel += ECD_PER_TURN;
    }

    return (float)rel * ECD_TO_RAD;
}

/* ------------------------------ ESO 控制器 ------------------------------ */

static void axis_ctrl_clear(axis_ctrl_t *c)
{
    c->st.eso_rate  = 0.0f;
    c->st.eso_dist  = 0.0f;
    c->st.comp_lpf  = 0.0f;
    c->st.integral  = 0.0f;
    c->st.out_last  = 0.0f;
    c->ff.out    = 0.0f;
    c->st.out_last_total = 0.0f;
    c->st.big_rate  = 0.0f;
    c->st.big_accel = 0.0f;
    c->st.rate_lpf  = 0.0f;
    c->st.rate_last = 0.0f;

    c->st.ref_last     = 0.0f;
    c->st.ref_shaped   = 0.0f;
    c->st.ref_rate_lpf = 0.0f;
    c->st.started      = 0;
    c->cal.test_current = 0.0f;         /* 停机时清掉，避免残留固定电流 */
    c->cal.test_sq_amp  = 0.0f;         /* 同上，重新解锁不会突然起步 */
    c->cal.test_sq_t    = 0.0f;
}

static void axis_ctrl_init(axis_ctrl_t *c)
{
    c->cfg.b0   = GIMBAL_6020_B0;
    c->cfg.wc   = GIMBAL_6020_WC;
    c->cfg.zeta = GIMBAL_6020_ZETA;
    c->cfg.rate_lpf_hz  = GIMBAL_6020_WLP;

    c->st.kp = c->cfg.wc * c->cfg.wc / c->cfg.b0;
    c->st.kd = 2.0f * c->cfg.zeta * c->cfg.wc / c->cfg.b0;

    c->cfg.ki         = GIMBAL_6020_KI_INT;
    c->cfg.i_limit    = GIMBAL_6020_ILIM;
    c->cfg.eso_bw     = GIMBAL_6020_ESO_WO;
    c->cfg.eso_comp   = GIMBAL_6020_ESO_GAIN;
    c->cfg.comp_lpf_hz = GIMBAL_6020_COMP_LPF;
    c->cfg.dist_limit = GIMBAL_6020_DLIM;
    c->cfg.out_limit  = GIMBAL_6020_CLIM;
    c->cfg.slew_limit = GIMBAL_6020_CSLEW;
    c->cfg.ref_lpf_hz = GIMBAL_6020_WREF_LP;
    c->cfg.ref_slew   = GIMBAL_6020_REF_SLEW;

    /* 耦合前馈默认值；Watch 里可临时改 */
    c->ff.enable  = GIMBAL_6020_FF_ENABLE;
    c->ff.sign    = GIMBAL_6020_FF_SIGN;
    c->ff.mass  = GIMBAL_6020_LOAD_MASS;
    c->ff.r     = GIMBAL_6020_LOAD_R;
    c->ff.d     = GIMBAL_6020_AXIS_D;
    c->ff.alpha = GIMBAL_6020_LOAD_ALPHA;
    c->cal.test_current = 0.0f;
    c->cal.test_sq_amp    = 0.0f;       /* 默认关，Watch 里开 */
    c->cal.test_sq_period = GIMBAL_6020_TSQ_PERIOD;

    axis_ctrl_clear(c);
}

/* --------------------------- 控制流水线 ---------------------------
 * 顺序不能变：
 *   增益 → 速度低通 → 参考整形 → ESO → 误差 → 积分 → 补偿低通
 *   → 反馈 → 标定注入 → 前馈 → 限幅 → 记账
 *
 * ESO 只建模反馈电流：out_last = 总输出 − out_ff。
 * 前馈如果进了 ESO 模型，会被补偿一次再被前馈抵消一次，变成双重补偿。
 * ----------------------------------------------------------------- */

/* 每次重算，b0 / wc / zeta 才能在线改 */
static void update_gains(axis_ctrl_t *c)
{
    c->st.kp = c->cfg.wc * c->cfg.wc / c->cfg.b0;
    c->st.kd = 2.0f * c->cfg.zeta * c->cfg.wc / c->cfg.b0;
}

static float filter_rate(axis_ctrl_t *c, float y_rate, float dt)
{
    if (c->cfg.rate_lpf_hz <= 0.0f)
    {
        c->st.rate_lpf = y_rate;
        return y_rate;
    }

    c->st.rate_lpf += lp_alpha(c->cfg.rate_lpf_hz, dt) * (y_rate - c->st.rate_lpf);
    return c->st.rate_lpf;
}

/* 参考限速 + 参考角速度前馈，返回 ref_rate_lpf */
static float shape_reference(axis_ctrl_t *c, float ref, float dt)
{
    float ref_rate;

    /* 阶跃/大跨距变成限速斜坡，保证 kp*err 不超力矩预算 */
    if (c->st.started == 0 || c->cfg.ref_slew <= 0.0f)
    {
        c->st.ref_shaped = ref;
    }
    else
    {
        float step = c->cfg.ref_slew * dt;

        c->st.ref_shaped += clampf(ref - c->st.ref_shaped, -step, step);
    }

    /* 差分目标角，限幅 + 低通 */
    ref_rate = c->st.started ? (c->st.ref_shaped - c->st.ref_last) / dt : 0.0f;
    c->st.started  = 1;
    c->st.ref_last = c->st.ref_shaped;

    ref_rate = clampf(ref_rate, -GIMBAL_6020_WREF_LIM, GIMBAL_6020_WREF_LIM);

    if (c->cfg.ref_lpf_hz > 0.0f)
    {
        c->st.ref_rate_lpf += lp_alpha(c->cfg.ref_lpf_hz, dt) * (ref_rate - c->st.ref_rate_lpf);
    }
    else
    {
        c->st.ref_rate_lpf = ref_rate;
    }

    return c->st.ref_rate_lpf;
}

/* ESO：估计总扰动。模型输入只用上一拍反馈电流 out_last */
static void update_eso(axis_ctrl_t *c, float rate, float dt)
{
    if (c->cfg.eso_bw <= 0.0f)
    {
        c->st.eso_rate = rate;
        c->st.eso_dist = 0.0f;
        return;
    }

    {
        float eso_err = c->st.eso_rate - rate;
        float b1      = 2.0f * c->cfg.eso_bw;
        float b2      = c->cfg.eso_bw * c->cfg.eso_bw;

        c->st.eso_rate += (c->st.eso_dist + c->cfg.b0 * c->st.out_last - b1 * eso_err) * dt;
        c->st.eso_dist += (-b2 * eso_err) * dt;
        c->st.eso_dist = clampf(c->st.eso_dist, -c->cfg.dist_limit, c->cfg.dist_limit);
    }
}

/* 误差取 ±π 最短路径 */
static float wrap_pi(float angle)
{
    if (angle > PI_F)
    {
        return angle - TWO_PI_F;
    }
    if (angle < -PI_F)
    {
        return angle + TWO_PI_F;
    }
    return angle;
}

static void update_integral(axis_ctrl_t *c, float err, float dt)
{
    c->st.integral = clampf(c->st.integral + c->cfg.ki * err * dt, -c->cfg.i_limit, c->cfg.i_limit);
}

/* 只滤补偿量，不滤观测器本身。不滤会把机械共振泵起来（实测 wo=80 激起 ~20Hz）*/
static void update_comp_lpf(axis_ctrl_t *c, float dt)
{
    if (c->cfg.eso_bw <= 0.0f)
    {
        c->st.comp_lpf = 0.0f;
        return;
    }

    if (c->cfg.comp_lpf_hz > 0.0f)
    {
        c->st.comp_lpf += lp_alpha(c->cfg.comp_lpf_hz, dt) * (c->st.eso_dist - c->st.comp_lpf);
    }
    else
    {
        c->st.comp_lpf = c->st.eso_dist;
    }
}

/* 状态反馈 + 速度前馈 + 积分 − ESO 补偿 */
static float compute_feedback(const axis_ctrl_t *c, float err,
                              float rate, float ref_rate)
{
    return c->st.kp * err
         + c->st.kd * (ref_rate - rate)
         + c->st.integral
         - c->st.comp_lpf / c->cfg.b0 * c->cfg.eso_comp;
}

/* 调试辨识：固定电流 + 标定方波。正式运行全 0 */
static float apply_calibration(axis_ctrl_t *c, float out, float dt)
{
    out += c->cal.test_current;

    /* 速度三角波斜率 = b0 × 电流。用前先归零 wc/ki/eso/slew */
    if (c->cal.test_sq_amp != 0.0f)
    {
        float amp    = clampf(c->cal.test_sq_amp, -GIMBAL_6020_TSQ_LIM, GIMBAL_6020_TSQ_LIM);
        float period = (c->cal.test_sq_period > 2.0f) ? c->cal.test_sq_period : 2.0f;

        c->cal.test_sq_t += dt * 1000.0f;

        if (c->cal.test_sq_t >= period)
        {
            c->cal.test_sq_t -= period;
        }

        out += (c->cal.test_sq_t < period * 0.5f) ? amp : -amp;
    }

    return out;
}

/* 大 yaw 耦合前馈：抵消 a_couple，只加在物理输出上 */
static float compute_ff(axis_ctrl_t *c, float y_pos)
{
    float phase;
    float arm;
    float accel;
    float ff;

    if (c->ff.enable == 0 || c->cal.test_sq_amp != 0.0f)   /* 标定时关掉，避免污染 b0 */
    {
        return 0.0f;
    }

    phase = y_pos + c->ff.alpha;
    arm   = c->ff.mass * c->ff.d * c->ff.r;
    accel = c->st.big_accel * arm_cos_f32(phase)
          + c->st.big_rate * c->st.big_rate * arm_sin_f32(phase);

    /* 第一项消 −φ̈_b，第二项消 −k[...]；第二项与 b0 无关 */
    ff = c->ff.sign
       * (c->st.big_accel / c->cfg.b0
          + arm / GIMBAL_6020_KT_EFF * accel);

    return clampf(ff, -c->cfg.out_limit, c->cfg.out_limit);
}

static float apply_output_limits(axis_ctrl_t *c, float out, float ff, float dt)
{
    /* 物理输出 = 反馈 + 前馈 */
    out = clampf(out + ff, -c->cfg.out_limit, c->cfg.out_limit);

    if (c->cfg.slew_limit > 0.0f)
    {
        float dmax = c->cfg.slew_limit * dt;

        out = clampf(out, c->st.out_last_total - dmax, c->st.out_last_total + dmax);
    }

    c->st.out_last_total = out;
    c->st.out_last       = out - ff;   /* ESO 只看到反馈部分 */

    return out;
}

/* 返回电流给定 counts（已含 out_turn）*/
static float axis_ctrl_step(axis_ctrl_t *c, float ref, float y_pos, float y_rate,
                            float dt, float out_turn)
{
    float err;
    float out;
    float ff;
    float rate;
    float ref_rate;

    update_gains(c);

    rate     = filter_rate(c, y_rate, dt);
    ref_rate = shape_reference(c, ref, dt);

    update_eso(c, rate, dt);

    err = wrap_pi(c->st.ref_shaped - y_pos);

    update_integral(c, err, dt);
    update_comp_lpf(c, dt);

    out = compute_feedback(c, err, rate, ref_rate);
    out = apply_calibration(c, out, dt);

    ff = compute_ff(c, y_pos);
    c->ff.out = ff;

    out = apply_output_limits(c, out, ff, dt);
    c->st.rate_last = y_rate;

    return out * out_turn;
}

/* ------------------------------ 电机接口 ------------------------------ */

void gimbal_6020_init(gimbal_6020_t *motor, CAN_HandleTypeDef *hcan)
{
    motor->hcan = hcan;

#if GIMBAL_6020_OFFSET_ENABLE
    motor->offset_ecd = GIMBAL_6020_OFFSET_ECD;
#endif

    axis_ctrl_init(&motor->ctrl);
}

void gimbal_6020_reset(gimbal_6020_t *motor)
{
    axis_ctrl_clear(&motor->ctrl);
}

uint8_t gimbal_6020_decode(gimbal_6020_t *motor, uint32_t std_id, const uint8_t *data)
{
    if (std_id != GIMBAL_6020_RX_ID)
    {
        return 0;
    }

    motor->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

#if (GIMBAL_6020_OFFSET_ENABLE == 0)
    if (motor->online == 0)
    {
        motor->offset_ecd = motor->ecd;
    }
#endif

    motor->angle = ecd_to_angle(motor->ecd, motor->offset_ecd);
    motor->online = 1;

    return 1;
}

void gimbal_6020_set_current(gimbal_6020_t *motor, int16_t current)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8] = {0};

    /* 没收到过反馈就不发，避免把 CAN 打到 Bus-Off */
    if (motor->online == 0)
    {
        return;
    }

    tx_data[GIMBAL_6020_TX_SLOT]     = (uint8_t)(current >> 8);
    tx_data[GIMBAL_6020_TX_SLOT + 1] = (uint8_t)(current);

    tx_message.StdId = GIMBAL_6020_TX_ID;
    tx_message.ExtId = 0;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 8;
    tx_message.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(motor->hcan, &tx_message, tx_data, &send_mail_box);
}

void gimbal_6020_set_angle(gimbal_6020_t *motor, float angle_set, float y_rate)
{
    float out;

    if (motor->online == 0)
    {
        return;
    }

    angle_set = clampf(angle_set, -PI_F, PI_F);

    out = axis_ctrl_step(&motor->ctrl, angle_set, motor->angle, y_rate,
                         GIMBAL_6020_DT, GIMBAL_6020_OUT_TURN);

    gimbal_6020_set_current(motor, (int16_t)out);
}
