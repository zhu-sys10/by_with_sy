/**
  * @file   yaw_aim.c
  * @brief  世界角瞄准：IMU 给绝对世界角，小 yaw 锁角，大 yaw 回中
  */

#include "yaw_aim.h"
#include "by_with_sy.h"
#include "user_lib.h"
#include "arm_math.h"   /* PI */

#define DEG_TO_RAD  (0.0174532925f)

volatile float yaw_aim_cmd_deg    = YAW_AIM_CMD_IDLE;   /* 调试写入口 */
volatile float yaw_aim_target_rad = 0.0f;               /* 当前世界目标 */

static float s_big_ref;   /* 大 yaw 目标 rad */

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

void yaw_aim_enter(float world_yaw, float big_yaw)
{
    yaw_aim_target_rad = rad_format(world_yaw);
    s_big_ref          = big_yaw;
}

void yaw_aim_set_target(float world_yaw)
{
    /* 不 wrap：目标要能连续累加，大 yaw 才会一直朝同一方向回中 */
    yaw_aim_target_rad = clampf(world_yaw, -YAW_BIG_MAX_RAD, YAW_BIG_MAX_RAD);
}

float yaw_aim_get_target(void)
{
    return yaw_aim_target_rad;
}

void yaw_aim_update(float dt, float world_yaw, float small_yaw, float big_yaw,
                    float big_manual_rate, float *big_cmd, float *big_rate,
                    float *small_cmd)
{
    float err_world;
    float small_raw;
    float big_target;
    float rate;

    
    if (yaw_aim_cmd_deg != YAW_AIM_CMD_IDLE)
    {
        yaw_aim_set_target(yaw_aim_cmd_deg * DEG_TO_RAD);
        yaw_aim_cmd_deg = YAW_AIM_CMD_IDLE;
    }

    /* ---------- 世界误差：以 IMU 为准 ---------- */
    err_world = rad_format(yaw_aim_target_rad - world_yaw);

    /* ---------- 小 yaw：吃掉全部世界误差 ---------- */
    /* 小 yaw 转多少，世界角就转多少，所以直接把误差加在当前位置上 */
    small_raw  = small_yaw + err_world;
    *small_cmd = clampf(small_raw, -YAW_AIM_SMALL_LIMIT, YAW_AIM_SMALL_LIMIT);

    /* ---------- 大 yaw 目标：小 yaw 回到 0 时大 yaw 该在哪 ---------- */
    /* 大 yaw 自己转动时本式不变（大 yaw 涨多少，世界误差就掉多少），不会自激 */
    big_target = big_yaw + small_raw;

    /* ---------- 大 yaw：摇杆优先，否则自动回中 ---------- */
    if (big_manual_rate != 0.0f)
    {
        rate = big_manual_rate;
    }
    else
    {
        rate = clampf(YAW_AIM_CENTER_GAIN * rad_format(big_target - s_big_ref),
                      -YAW_AIM_CENTER_SLEW,
                      YAW_AIM_CENTER_SLEW);
    }

    /* 位置被限幅时参考速度必须归零，否则速度前馈会继续把轴推出去 */
    s_big_ref += rate * dt;
    if (s_big_ref > YAW_BIG_MAX_RAD)
    {
        s_big_ref = YAW_BIG_MAX_RAD;
        rate      = 0.0f;
    }
    else if (s_big_ref < -YAW_BIG_MAX_RAD)
    {
        s_big_ref = -YAW_BIG_MAX_RAD;
        rate      = 0.0f;
    }

    *big_cmd  = s_big_ref;
    *big_rate = rate;
}
