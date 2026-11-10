/**
  * @file   dm_motor.h
  * @brief  达妙 DM 系列电机 MIT 模式驱动
  */

#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "can.h"

/* 量程，必须和调试助手一致。实测助手：PMAX=3.14159 / VMAX=30 / TMAX=10 */
#define DM_P_MIN  (-3.1415927f)
#define DM_P_MAX  (3.1415927f)
#define DM_V_MIN  (-30.0f)
#define DM_V_MAX  (30.0f)
#define DM_T_MIN  (-10.0f)
#define DM_T_MAX  (10.0f)
#define DM_KP_MIN (0.0f)
#define DM_KP_MAX (500.0f)
#define DM_KD_MIN (0.0f)
#define DM_KD_MAX (5.0f)

/* pos 反馈的锯齿周期：过 ±π 翻转，一格 2π */
#define DM_P_SPAN (DM_P_MAX - DM_P_MIN)

/* vel 字段修正系数。实测 PMAX 填错导致之前的 3.95 是位置量程误差，
   速度字段本身正常，保持 1.0 */
#define DM_V_SCALE (1.0f)

#define DM_CMD_ENABLE  (0xFCu)      /* 命令帧 = 7 字节 0xFF + 命令字 */
#define DM_CMD_DISABLE (0xFDu)

#define DM_POS_KP_DEFAULT (15.0f)   /* 位置刚度 */
#define DM_POS_KD_DEFAULT (1.0f)    /* 阻尼，不能为 0 */
#define DM_START_DELAY_MS (500u)    /* 上电延时 */

#define DM_ERR_ENABLED     (0x01u)  /* 反馈帧 D0 高 4 位状态码 */
#define DM_ERR_IS_FAULT(e) (((e) >= 0x08u) && ((e) <= 0x0Eu))

typedef struct
{
    CAN_HandleTypeDef *hcan;

    uint8_t can_id;                 /* 控制帧 ID */
    uint8_t master_id;              /* 反馈帧 ID */

    float   kp;                     /* 位置刚度 */
    float   kd;                     /* 阻尼 */

    float   pos;                    /* rad，协议原始值，±π 处翻转 */
    float   pos_cont;               /* rad，按 25 rad 展开后的连续角 */
    float   pos_last;               /* rad，上一帧原始值，展开用 */
    float   vel;                    /* rad/s */
    float   torque;                 /* N.m */
    uint8_t t_mos;                  /* ℃ */
    uint8_t t_rotor;                /* ℃ */
    uint8_t err;                    /* D0 高 4 位状态码 */
    uint8_t online;                 /* 1 = 收到过反馈 */
} dm_motor_t;

void dm_motor_init(dm_motor_t *motor, CAN_HandleTypeDef *hcan, uint8_t can_id, uint8_t master_id);
void dm_motor_enable(dm_motor_t *motor);            /* 上电必须发 */
void dm_motor_disable(dm_motor_t *motor);           /* 扭矩归零 */
void dm_motor_mit(dm_motor_t *motor, float pos);    /* 位置控制，给目标角度 */
void dm_motor_mit_full(dm_motor_t *motor, float pos, float vel,
                       float kp, float kd, float t_ff);  /* 完整 MIT 指令 */

/* 解析反馈帧，1 = 本电机的帧 */
uint8_t dm_motor_decode(dm_motor_t *motor, uint32_t std_id, const uint8_t *data);

#endif
