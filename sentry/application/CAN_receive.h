/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

#define ANGLE_T 8191
extern float PowerData[4];
/* CAN send and receive ID */
typedef enum
{
//CAN1 ID
  CAN_CHASSIS_ALL_ID = 0x200,
  CAN_3508_M1_ID ,
  CAN_3508_M2_ID = 0x202,
  CAN_3508_M3_ID = 0x203,
  CAN_3508_M4_ID = 0x204,
	
  CAN_CHASSIS = 0x001,//receive from radar

  CAN_NAVIGATION = 0x002,//receive from master
  CAN_MESSAGE = 0x003,//receivr from master
  CAN_MESSAGE2 = 0x004, //receive from master

  //CAN_MASTER_MESSAGE = 0x005,//receive from master

  CAN_GIMBAL_YAW_ID = 0x2FF,
  CAN_GIMBAL_PIT_ID = 0x4FE,
  CAN_YAW_MOTOR_ID = 0x209,
  CAN_PIT_MOTOR_ID = 0x306,
  CAN_TRIGGER_MOTOR_ID = 0x207,

//CAN2 ID
  CAN_FRICTION_ALL_ID = 0x200,
  CAN_LEFT_FRICTION_ID = 0x201,
  CAN_RIGHT_FRICTION_ID = 0x202  
} can_msg_id_e;

// rm motor data
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
  int32_t round;
  float angle;
} motor_measure_t;

extern void CAN_cmd_navigation(void);
extern void CAN_cmd_decesion(void);
extern void CAN_cmd_decesion2(void);

/**
 * @brief 摩擦轮控制量发送函数 can2
 *
 * @param 左摩擦轮电流
 * @param 右摩擦轮电流
 */
extern void CAN_cmd_fricion(int16_t left_friction, int16_t rigit_friction);
extern void CAN_cmd_NUC(uint8_t buf[8]);


/**
 * @brief 拨弹电机控制量发送函数 can1
 *
 * @param shoot
 */
extern void CAN_cmd_shoot(int16_t shoot);

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
extern void CAN_cmd_gimbal_yaw(int16_t yaw);
extern void CAN_cmd_gimbal_pitch(int16_t pitch);
/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_radar_gimbal_motor_measure_point(void);

extern const motor_measure_t *get_left_friction_motor_measure_point(void);
extern const motor_measure_t *get_right_friction_motor_measure_point(void);

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

typedef struct
{
  float vx_set;
  float vy_set;
  float wz_set;
  uint8_t wz_flag;
	int ramp_flag;
	bool_t new_chassis_data_flag;
} auto_chassis_t;

typedef struct
{
uint8_t game_progress;
uint16_t stage_remain_time;
uint16_t     remain_HP;
uint16_t bullet_remaining_num;
uint8_t robot_id;
uint16_t blue_base_HP;
uint16_t blue_outpost_HP;
uint16_t red_base_HP;
uint16_t red_outpost_HP;
uint8_t RFID_Base;
}game_mesage_t;

extern auto_chassis_t auto_chassis;

#endif
