/**
 * @file     shoot_task.h
 * @author   何清华，lkhannn
 * @brief    上下双枪哨兵的摩擦轮、拨盘控制
 * @version  1.0
 * @date     2024-5-4
 * @copyright Copyright (c) 2024 SPR
 *
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __shoot_TASK_H__
#define __shoot_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME        300
//射击任务控制间隔 5ms
#define SHOOT_CONTROL_TIME          5
//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       RIGHT_SWITCH
//发射机构pid参数
//摩擦轮
#define UP_FRICTION_KP 29.3f
#define UP_FRICTION_KD 0.0f
#define UP_FRICTION_KI 0.29f
#define UP_FRICTION_MAX_OUT 16000.0f
#define UP_FRICTION_MAX_IOUT 0.0f
//拨盘
#define TRIGGER_KP 15.0f//17.5f 
#define TRIGGER_KI 0.0f//0.3f
#define TRIGGER_KD 0.0f
#define TRIGGER_MAX_OUT 10000.0f
#define TRIGGER_MAX_IOUT 3500.0f
//拨弹速度
#define TRIGGER_SPEED               -10000.0f * 2.22251415*0.24*2.0f//10Hz过检录

#define AUTO_TRIGGER_SPEED			-3500.0f * 3.0f

//摩擦轮速
#define FRICTION_SPEED_SET          -6000.0f

#define FRICTION_ACCEL_MAX_OUT      3000//摩擦轮启动加速电流限幅
#define FRICTION_MAX_OUT            8000//持续发射电流限幅，16384是20A，4000即4.88A

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  150 //5 * 150 = 750ms
#define REVERSE_TIME                100 //5 * 100 = 500ms

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,//开摩擦轮
    SHOOT_BULLET,//子弹
	SHOOT_SINGLE_BULLET,
} shoot_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float speed;
    float speed_set;
    float angle;
    float set_angle;
    int16_t given_current;
} trigger_motor_t;

typedef struct
{
    motor_measure_t motor_measure;
    pid_type_def motor_speed_pid;
    float speed_set;      //设定速度
	float speed;          //当前速度
	float accel;          //加速度
    int16_t given_current; //发送电流
} general_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;

    trigger_motor_t trigger_motor;
	general_motor_t left_friction;
	general_motor_t right_friction;
	
    uint16_t block_time;
    uint16_t reverse_time;

    uint16_t heat_limit;
    uint16_t heat0;
	uint16_t heat1;
	
	float shoot_speed1;
	float shoot_speed2;
	int shoot_level;		//和自瞄控制距离有关

} shoot_control_t;

void fire_task(void const *pvParameters);

extern shoot_control_t shoot_control;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
