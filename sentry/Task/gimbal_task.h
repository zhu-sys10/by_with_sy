/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/* ****************************************************************** 
                        AGV_CHASIS
****************************************************************** */
//任务初始化 空闲一段时间0
#define GIMBAL_TASK_INIT_TIME 201
//云台任务控制间隔 1ms
#define GIMBAL_CONTROL_TIME 1
// yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3 //LEFT Y
#define PITCH_CHANNEL_RIGHT 1
#define GIMBAL_MODE_CHANNEL LEFT_SWITCH
//遥控器遥杆
#define RC_LEFT_Y 3
#define RC_LEFT_X 2
#define RC_RIGHT_Y 1
#define RC_RIGHT_X 0
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 10
//灵敏度
#define YAW_RC_SEN -0.0000055f
#define PITCH_RC_SEN 0.0000053f//-0.0000053f
#define YAW_MOUSE_SEN 0.00004f//
#define PITCH_MOUSE_SEN 0.0001f//0.00015f
///////////////////////////电机、陀螺仪安装方向////////////////////////////////
#define PITCH_TURN 1
#define PITCH_OUT_TURN 1
#define YAW_TURN 0
#define YAW_OUT_TURN 1
#define YAW_TURN_GYRO 1
//PITCH绝对角度到角速度PID参数
#define PITCH_ABSOLUTE_ANGLE_KP 7.0f//15.0f//
#define PITCH_ABSOLUTE_ANGLE_KI 0.0099999998f//0.004f//0.001f
#define PITCH_ABSOLUTE_ANGLE_KD 0.899999976f//0.8f//0.8f
#define PITCH_ABSOLUTE_ANGLE_KF 0//500.0f//0.8f
#define PITCH_ABSOLUTE_ANGLE_MAX_OUT 5.0f//20.0f
#define PITCH_ABSOLUTE_ANGLE_MAX_IOUT 0.3f//1.0f
//PITCH相对角度到角速度PID参数
#define PITCH_RELATIVE_ANGLE_KP 8.0f//1.0f//
#define PITCH_RELATIVE_ANGLE_KI 0.0f//0.001f//0.0001f
#define PITCH_RELATIVE_ANGLE_KD 0.0f//0.0f//0.0f
#define PITCH_RELATIVE_ANGLE_KF 0//0.0f//0.0f
#define PITCH_RELATIVE_ANGLE_MAX_OUT 5.0f//20.0f
#define PITCH_RELATIVE_ANGLE_MAX_IOUT 0.3f//1.0f

//PITCH角速度到电流PID参数
#define PITCH_GYRO_KP 900//4000.0f
#define PITCH_GYRO_KI 5
#define PITCH_GYRO_KD 300
#define PITCH_GYRO_MAX_OUT 2000.0f//9000.0f//25000.0f
#define PITCH_GYRO_MAX_IOUT 0.0f
//Yaw云台PID参数，遥控
#define YAW_ABSOLUTE_ANGLE_KP 22.0f	
#define YAW_ABSOLUTE_ANGLE_KI 0.0f//0.035f
#define YAW_ABSOLUTE_ANGLE_KD -1.0f
#define YAW_ABSOLUTE_ANGLE_KF 1300.0f
#define YAW_ABSOLUTE_ANGLE_MAX_OUT 22.0f//15
#define YAW_ABSOLUTE_ANGLE_MAX_IOUT 0.3f//2.0f
//Yaw角速度到电流PID参数
#define YAW_GYRO_KP 7000.0f//12000.0f
#define YAW_GYRO_KI 0.0f
#define YAW_GYRO_KD 6500.0f//3200.0f//0.0f
#define YAW_GYRO_MAX_OUT 22000.0f///22000.0f
#define YAW_GYRO_MAX_IOUT 0.0f
//云台限位及中值参数设定
#define GIMBAL_YAW_OFFSET_ECD 5153		//看yaw轴ecd
#define GIMBAL_YAW_MAX_ECD 3.12f		//不改
#define GIMBAL_YAW_MIN_ECD -3.12f		//不改
#define GIMBAL_PITCH_OFFSET_ECD 490	//看pitch轴ecd
#define GIMBAL_PITCH_MAX_AST 0.253//0.35f		//看absolute_angle
#define GIMBAL_PITCH_MIN_AST -0.297//-0.35f		//看absolute_angle

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
  const motor_measure_t *gimbal_motor_measure;
  gimbal_PID_t gimbal_motor_absolute_angle_pid;
  gimbal_PID_t gimbal_motor_relative_angle_pid;
  pid_type_def gimbal_motor_gyro_pid;
  gimbal_motor_mode_e gimbal_motor_mode;
  float offset_ecd;
  float max_relative_angle; // rad
  float min_relative_angle; // rad

  float relative_angle;     // rad
  float relative_angle_set; // rad
  float absolute_angle;     // rad
  float absolute_angle_set; // rad
  float motor_gyro;         // rad/s 
  float motor_gyro_set;
  float motor_speed;
  float current_set;
  int16_t given_current;
} gimbal_motor_t;

typedef struct
{
  gimbal_motor_t gimbal_yaw_motor;
  gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

typedef struct
{
  uint16_t last_remain_HP;
  uint8_t chassis_start_top_flag;//0为底盘不旋转，1为底盘旋转
  uint16_t chassis_turn_time;
  uint16_t gimbal_turn_time;
  float chassis_wz_speed;
}chassis_and_gimbal_t;
extern chassis_and_gimbal_t chassis_and_gimbal;

extern gimbal_control_t gimbal_control;

/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

extern void gimbal_task(void const *pvParameters);

#endif
