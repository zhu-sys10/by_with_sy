/**
  * @file   dm_motor.c
  * @brief  达妙 DM 系列电机 MIT 模式驱动
  */

#include "dm_motor.h"

/* 超量程要先限幅，否则转定点时位域会翻转 */
static int32_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;

    if (x < x_min)
    {
        x = x_min;
    }
    if (x > x_max)
    {
        x = x_max;
    }

    return (int32_t)((x - x_min) * (float)((1 << bits) - 1) / span);
}

static float dm_uint_to_float(int32_t x, float x_min, float x_max, uint8_t bits)
{
    return (float)x * (x_max - x_min) / (float)((1 << bits) - 1) + x_min;
}

/* 命令帧：FF FF FF FF FF FF FF <cmd> */
static void dm_motor_send_cmd(dm_motor_t *motor, uint8_t cmd)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    tx_data[7] = cmd;

    tx_message.StdId = motor->can_id;
    tx_message.ExtId = 0;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 8;
    tx_message.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(motor->hcan, &tx_message, tx_data, &send_mail_box);
}

void dm_motor_init(dm_motor_t *motor, CAN_HandleTypeDef *hcan, uint8_t can_id, uint8_t master_id)
{
    motor->hcan = hcan;
    motor->can_id = can_id;
    motor->master_id = master_id;
    motor->kp = DM_POS_KP_DEFAULT;
    motor->kd = DM_POS_KD_DEFAULT;
}

void dm_motor_enable(dm_motor_t *motor)
{
    dm_motor_send_cmd(motor, DM_CMD_ENABLE);
}

void dm_motor_disable(dm_motor_t *motor)
{
    dm_motor_send_cmd(motor, DM_CMD_DISABLE);
}

void dm_motor_mit_full(dm_motor_t *motor, float pos, float vel,
                       float kp, float kd, float t_ff)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t tx_data[8];
    uint16_t p_int;
    uint16_t v_int;
    uint16_t kp_int;
    uint16_t kd_int;
    uint16_t t_int;

    p_int  = (uint16_t)dm_float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    v_int  = (uint16_t)dm_float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_int = (uint16_t)dm_float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_int = (uint16_t)dm_float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    t_int  = (uint16_t)dm_float_to_uint(t_ff, DM_T_MIN, DM_T_MAX, 12);

    tx_data[0] = (uint8_t)(p_int >> 8);
    tx_data[1] = (uint8_t)(p_int);
    tx_data[2] = (uint8_t)(v_int >> 4);
    tx_data[3] = (uint8_t)((v_int << 4) | (kp_int >> 8));
    tx_data[4] = (uint8_t)(kp_int);
    tx_data[5] = (uint8_t)(kd_int >> 4);
    tx_data[6] = (uint8_t)((kd_int << 4) | (t_int >> 8));
    tx_data[7] = (uint8_t)(t_int);

    tx_message.StdId = motor->can_id;
    tx_message.ExtId = 0;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 8;
    tx_message.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(motor->hcan, &tx_message, tx_data, &send_mail_box);
}

void dm_motor_mit(dm_motor_t *motor, float pos)
{
    dm_motor_mit_full(motor, pos, 0.0f, motor->kp, motor->kd, 0.0f);
}

uint8_t dm_motor_decode(dm_motor_t *motor, uint32_t std_id, const uint8_t *data)
{
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    float    pos_raw;
    float    dpos;

    if (std_id != (uint32_t)motor->master_id)
    {
        return 0;
    }

    motor->err = (uint8_t)(data[0] >> 4);       /* D0 高 4 位 */

    p_int = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    v_int = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    t_int = (uint16_t)((((uint16_t)data[4] & 0x0F) << 8) | data[5]);

    pos_raw = dm_uint_to_float((int32_t)p_int, DM_P_MIN, DM_P_MAX, 16);

    /* 第一帧直接对齐；之后跨过 ±12.5 翻转点时补/减一格 25 rad */
    if (motor->online == 0)
    {
        motor->pos_cont = pos_raw;
    }
    else
    {
        dpos = pos_raw - motor->pos_last;
        if (dpos > DM_P_MAX)        /* 越过 +12.5，翻到了 -12.5 */
        {
            motor->pos_cont -= DM_P_SPAN;
        }
        else if (dpos < DM_P_MIN)   /* 越过 -12.5，翻到了 +12.5 */
        {
            motor->pos_cont += DM_P_SPAN;
        }
        motor->pos_cont += dpos;
    }
    motor->pos_last = pos_raw;

    motor->pos    = pos_raw;
    motor->vel    = dm_uint_to_float((int32_t)v_int, DM_V_MIN, DM_V_MAX, 12) * DM_V_SCALE;
    motor->torque = dm_uint_to_float((int32_t)t_int, DM_T_MIN, DM_T_MAX, 12);

    motor->t_mos   = data[6];
    motor->t_rotor = data[7];
    motor->online  = 1;

    return 1;
}
