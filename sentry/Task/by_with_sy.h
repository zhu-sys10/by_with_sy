/**
  * @file   by_with_sy.h
  * @brief  双 yaw 台架任务：手动 / 世界角瞄准
  */

#ifndef __BY_WITH_SY_H__
#define __BY_WITH_SY_H__

#include "dm_motor.h"
#include "gimbal_6020.h"

#define DM_4310_CAN_ID    (0x05u)   /* 大 yaw 控制帧 ID */
#define DM_4310_MASTER_ID (0x55u)   /* 大 yaw 反馈帧 ID */

#define YAW_BIG_MAX_RAD   (3.1415927f)  /* 大 yaw 行程 ±180°（电机反馈量程 ±π） */
#define YAW_SMALL_MAX_RAD (1.05f)       /* 小 yaw 行程 ±60° */

/* 大 yaw 传动系数：yaw 轴转角 = 电机输出角 × 本值。
   手转 90° 实测：编码器 1.571 rad ↔ 世界角 90°，直驱填 1.0。 */
#define YAW_BIG_RATIO     (1.0f)

/* 大 yaw 机械零位对应的 dm_4310.pos（rad）。
   旧值 -5.93 是 PMAX 填错 12.5 时的读数，× π/12.5 = -1.49。 */
#define BIG_START_FIXED   (-1.49f)

extern dm_motor_t    dm_4310;
extern gimbal_6020_t g6020;

extern volatile float yaw_big_set;      /* 大 yaw 目标 rad（相对上电位置）*/
extern volatile float yaw_small_set;    /* 小 yaw 目标 rad（相对大 yaw）*/
extern volatile uint8_t big_yaw_free;   /* 1 = 大 yaw 零力矩，手转标定用 */
extern volatile float yaw_big_vel;      /* 位置差分估计的电机角速度 rad/s，只读 */

extern volatile float yaw_big_angle;    /* 大 yaw 当前轴角 rad，只读 */
extern volatile float yaw_world_angle;  /* 世界角 rad（IMU），只读 */
extern volatile float yaw_enc_angle;    /* 编码器推算世界角，对比用 */
extern volatile float yaw_world_enc_diff; /* 世界角 − 编码器推算角，只读 */

void test_task(void const *argument);   /* 台架线程，1ms 一次 */

#endif
