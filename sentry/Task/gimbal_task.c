/**
 * @file gimbal_task.c/h
 * @author 何泰宇,何清华,周鸣阳
 * @brief 云台控制任务线程
 * @version 2.0
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024 SPR
 *
 */
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "cmsis_os.h"
#include "main.h"
#include "INS_task.h"
#include "arm_math.h"
#include "detect_task.h"
#include "user_lib.h"
#include "USART_receive.h"
#include "config.h"
#include "referee.h"
#include "chassis_task.h"

extern chassis_move_t chassis_move;

extern char if_dir_set;
extern float pitch_relative_angle;
extern gimbal_behaviour_e gimbal_behaviour;
extern void NUC_GG_Detector(void);

unsigned short NUC_GG_detect;//判断NUC是否掉线的变量
gimbal_control_t gimbal_control;//云台控制所有相关数据

int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;//发送的电机电流
//云台控制相关变量
float add_yaw_angle = 0.0f;
float add_pitch_angle = 0.0f;
float auto_yaw_target = 0.0f;
float auto_pitch_target = 0.0f;

chassis_and_gimbal_t chassis_and_gimbal;//受击打底盘与云台联动相关变量

// PID输出清零
#define gimbal_total_pid_clear(gimbal_clear)                                               \
  {                                                                                        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                           \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
  }
  

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @retval         none
 */
static void gimbal_init(void);

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @retval         none
 */
static void gimbal_set_mode(void);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @retval         none
 */
static void gimbal_feedback_update(void);

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @retval         none
 */
static void gimbal_set_control(void);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @retval         none
 */
static void gimbal_control_loop(void);

/**
 * @brief 云台姿态数据发送到上位机
 *
 */
void gimbal_data_send(void);

/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief 云台PID初始化函数
 *
 * @param pid PID数据结构指针
 * @param kp
 * @param ki
 * @param kd
 * @param maxout 最大输出
 * @param max_iout 最大积分输出
 */
static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float kf, float maxout, float max_iout);

/**
 * @brief 云台PID计算函数
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta 误差微分项，直接从传感器读取，不计算
 * @return fp32
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta);

/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

/**
 * @brief          如果电机安装方向相反，需要在电流值前添加负号
 * @param[out]     void
 * @retval         void
 */
static void gimbal_motor_turn(void);

void gimbal_task(void const *pvParameters)
{
  vTaskDelay(GIMBAL_TASK_INIT_TIME);

  //云台初始化
  gimbal_init();

  //判断电机是否都上线
//   while (toe_is_error(YAW_GIMBAL_MOTOR_TOE)||toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
//   {
//     gimbal_feedback_update(); //云台数据反馈
// 	gimbal_data_send();
//     vTaskDelay(GIMBAL_CONTROL_TIME);
//   }

  while (1)
  {
	//NUC掉线判断
	NUC_GG_Detector();
    //设置云台控制模式
    gimbal_set_mode();
    //云台数据反馈
    gimbal_feedback_update();
    //设置云台控制量
    gimbal_set_control();
    //云台控制PID
    gimbal_control_loop();
    //云台姿态数据发送到上位机
    gimbal_data_send();
	//云台电机发送电流方向控制	
	  gimbal_motor_turn();
	  
    if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
    {
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_gimbal_yaw(0);
		CAN_cmd_gimbal_pitch(0);
        //防止失控
        gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
		gimbal_PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
		PID_clear(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid);
      }
      else
      {
		  CAN_cmd_gimbal_yaw(GIMBAL_YAW_ZERO_CURRENT ? 0 : yaw_can_set_current);
		  CAN_cmd_gimbal_pitch(GIMBAL_PITCH_ZERO_CURRENT ? 0 : pitch_can_set_current);
      }
    }

    vTaskDelay(GIMBAL_CONTROL_TIME);
  }
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @retval         none
 */
static void gimbal_init(void)
{
  //电机数据指针获取
  gimbal_control.gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
  gimbal_control.gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
	
  //初始化电机模式
  gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
  gimbal_control.gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
	
  //初始化云台中值与限位
  gimbal_control.gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ECD;
  gimbal_control.gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_ECD;
  gimbal_control.gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_ECD;

  gimbal_control.gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_OFFSET_ECD;
  gimbal_control.gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_AST;
  gimbal_control.gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_AST;

  //初始化yaw电机pid
  gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, YAW_ABSOLUTE_ANGLE_KF, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //???????????
  PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid, YAW_GYRO_KP, YAW_GYRO_KI, YAW_GYRO_KD, YAW_GYRO_MAX_OUT, YAW_GYRO_MAX_IOUT);
  gimbal_PID_init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid, -YAW_ABSOLUTE_ANGLE_KP, YAW_ABSOLUTE_ANGLE_KI, YAW_ABSOLUTE_ANGLE_KD, YAW_ABSOLUTE_ANGLE_KF, YAW_ABSOLUTE_ANGLE_MAX_OUT, YAW_ABSOLUTE_ANGLE_MAX_IOUT); //???????????
  //初始化pitch电机pid
  gimbal_PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid, -PITCH_RELATIVE_ANGLE_KP, PITCH_RELATIVE_ANGLE_KI, PITCH_ABSOLUTE_ANGLE_KD, PITCH_RELATIVE_ANGLE_KD,PITCH_RELATIVE_ANGLE_MAX_OUT, PITCH_RELATIVE_ANGLE_MAX_IOUT); //???????????
  gimbal_PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_ABSOLUTE_ANGLE_KP, PITCH_ABSOLUTE_ANGLE_KI, PITCH_ABSOLUTE_ANGLE_KD, PITCH_ABSOLUTE_ANGLE_KF,PITCH_ABSOLUTE_ANGLE_MAX_OUT, PITCH_ABSOLUTE_ANGLE_MAX_IOUT);
  PID_init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid, PITCH_GYRO_KP, PITCH_GYRO_KI, PITCH_GYRO_KD, PITCH_GYRO_MAX_OUT, PITCH_GYRO_MAX_IOUT);
  //清除所有PID输出
  gimbal_total_pid_clear(&gimbal_control);
  //数据更新
  gimbal_feedback_update();
  //初始化启动状态
  gimbal_control.gimbal_yaw_motor.absolute_angle_set = gimbal_control.gimbal_yaw_motor.absolute_angle;
  gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_yaw_motor.relative_angle;
  gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_control.gimbal_yaw_motor.motor_gyro;

  gimbal_control.gimbal_pitch_motor.absolute_angle_set = gimbal_control.gimbal_pitch_motor.absolute_angle;
  gimbal_control.gimbal_pitch_motor.relative_angle_set = gimbal_control.gimbal_pitch_motor.relative_angle;
  gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_control.gimbal_pitch_motor.motor_gyro;

}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @retval         none
 */
static void gimbal_set_mode()
{
  gimbal_behaviour_mode_set(&gimbal_control);
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @retval         none
 */

static void gimbal_feedback_update()
{	
//如果编码器角度变化方向和陀螺仪角度变化方向相反，需要在相对角度前添加负号(保持相对角和绝对角变化一致)
#if PITCH_TURN
  gimbal_control.gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_pitch_motor.offset_ecd);
#else
  gimbal_control.gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_pitch_motor.offset_ecd);
#endif

  gimbal_control.gimbal_pitch_motor.absolute_angle = INS_data.angle_pitch;
  gimbal_control.gimbal_pitch_motor.motor_gyro = INS_data.wy;

#if YAW_TURN
  gimbal_control.gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
#else
  gimbal_control.gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
#endif

  gimbal_control.gimbal_yaw_motor.absolute_angle = INS_data.angle_yaw;

#if YAW_TURN_GYRO
  gimbal_control.gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wz - arm_sin_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wx;
  //gimbal_control.gimbal_yaw_motor.motor_gyro = INS_data.wz;
#else
  //直接用wz所得的是在当前俯仰角下，左右旋转的角速度，和我们想要的不符，需要进行分解即乘以一个旋转矩阵
  gimbal_control.gimbal_yaw_motor.motor_gyro = -(arm_cos_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wz - arm_sin_f32(gimbal_control.gimbal_pitch_motor.relative_angle) * INS_data.wx);
  //gimbal_control.gimbal_yaw_motor.motor_gyro = -INS_data.wz;
#endif
  
}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
//相对角度计算 目前是左正右负
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  if (relative_ecd > 4096)
  {
    relative_ecd -= 8191;
  }
  else if (relative_ecd < -4096)
  {
    relative_ecd += 8191;
  }
  return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @retval         none
 */

static void gimbal_set_control(void)
{
	gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    auto_yaw_target = gimbal_control.gimbal_yaw_motor.absolute_angle;
    auto_pitch_target = gimbal_control.gimbal_pitch_motor.absolute_angle;
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, &gimbal_control);
    gimbal_control.gimbal_yaw_motor.absolute_angle_set += add_yaw_angle;
	gimbal_control.gimbal_pitch_motor.absolute_angle_set += add_pitch_angle;
	



    
	if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE&&gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
	gimbal_relative_angle_limit(&gimbal_control.gimbal_pitch_motor, add_pitch_angle);
	}
	else
	{
	gimbal_absolute_angle_limit(&gimbal_control.gimbal_pitch_motor, add_pitch_angle);
	}
	
		if(gimbal_behaviour==GIMBAL_AUTO_GAME||gimbal_behaviour==GIMBAL_AUTO)//在自动模式下
		{
			if((chassis_and_gimbal.last_remain_HP-robot_state.remain_HP)>=10.0f||chassis_and_gimbal.gimbal_turn_time!=0)//当发生扣血时
			{
				if(chassis_and_gimbal.chassis_start_top_flag==0)//当底盘不旋转时
				{
					gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
					
					gimbal_control.gimbal_yaw_motor.absolute_angle_set=gimbal_control.gimbal_yaw_motor.absolute_angle;
					switch(robot_hurt_t.armor_type)//转向受击打的装甲板
					{ 
						case 0:
						{
							gimbal_control.gimbal_yaw_motor.relative_angle_set=-0.8f;//云台转向角度(弧度制)
							chassis_and_gimbal.gimbal_turn_time++;
							
							if(_abs(gimbal_control.gimbal_yaw_motor.relative_angle_set-gimbal_control.gimbal_yaw_motor.relative_angle)<=0.01f)
							{
							chassis_and_gimbal.gimbal_turn_time=1000;//云台旋转时间
							}
							break;
						}
						case 1:
						{
							gimbal_control.gimbal_yaw_motor.relative_angle_set=-2.4f;
							chassis_and_gimbal.gimbal_turn_time++;
							if(_abs(gimbal_control.gimbal_yaw_motor.relative_angle_set-gimbal_control.gimbal_yaw_motor.relative_angle)<=0.01f)
							{
							chassis_and_gimbal.gimbal_turn_time=1000;
							}
							break;
						}
						case 2:
						{
							gimbal_control.gimbal_yaw_motor.relative_angle_set=2.4f;
							chassis_and_gimbal.gimbal_turn_time++;
							if(_abs(gimbal_control.gimbal_yaw_motor.relative_angle_set-gimbal_control.gimbal_yaw_motor.relative_angle)<=0.01f)
							{
							chassis_and_gimbal.gimbal_turn_time=1000;
							}
							break;
						}
						case 3:
						{
							gimbal_control.gimbal_yaw_motor.relative_angle_set=0.8f;
							chassis_and_gimbal.gimbal_turn_time++;
							if(_abs(gimbal_control.gimbal_yaw_motor.relative_angle_set-gimbal_control.gimbal_yaw_motor.relative_angle)<=0.01f)
							{
							chassis_and_gimbal.gimbal_turn_time=1000;
							}
							break;
						}
						default:
						{
							break;
						}
					}
				}
			}
			if(chassis_and_gimbal.chassis_start_top_flag==1)//在开始旋转后，如果受击打或者回血则发生改变
			{
				if((chassis_and_gimbal.last_remain_HP-robot_state.remain_HP)>=10.0f)//如果旋转时受击打，则继续旋转
				{
					chassis_and_gimbal.chassis_turn_time=0;
				}
				if(robot_state.remain_HP-chassis_and_gimbal.last_remain_HP>0)//在回血时则不旋转
				{
					chassis_and_gimbal.chassis_turn_time=2500;
				}
				if(chassis_and_gimbal.chassis_turn_time==0)//从第一次扣血获得之后一段时间内的初始旋转速度
				{
					chassis_and_gimbal.chassis_wz_speed=(chassis_and_gimbal.last_remain_HP-robot_state.remain_HP)*0.15;
				}
			}
			chassis_and_gimbal.last_remain_HP=get_robot_HP();
			
			if(chassis_and_gimbal.gimbal_turn_time==1000)
			{
			chassis_and_gimbal.gimbal_turn_time=0;
			gimbal_control.gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
			chassis_and_gimbal.chassis_start_top_flag=1;
			}
		}
		else
		{
			chassis_and_gimbal.last_remain_HP=get_robot_HP();
		}
		
	if(if_dir_set & 1)//yaw直接赋控制值
		gimbal_control.gimbal_yaw_motor.absolute_angle_set = add_yaw_angle+chassis_move.wz/8.4f*0.04f;//底盘旋转有随着转速变换的稳定误差
	if(if_dir_set & 2)//pitch直接赋控制值 0000 0011&0000 0010=0000 0010即2,非零值即为真
		gimbal_control.gimbal_pitch_motor.absolute_angle_set = add_pitch_angle;
	if_dir_set = 0;	
	//pitch轴限位
//	if( gimbal_control.gimbal_pitch_motor.absolute_angle_set > GIMBAL_PITCH_MAX_AST )
//		gimbal_control.gimbal_pitch_motor.absolute_angle_set = GIMBAL_PITCH_MAX_AST;
//    if( gimbal_control.gimbal_pitch_motor.absolute_angle_set < GIMBAL_PITCH_MIN_AST )
//		gimbal_control.gimbal_pitch_motor.absolute_angle_set = GIMBAL_PITCH_MIN_AST;
}


/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
  static float bias_angle;
  static float angle_set;
  if (gimbal_motor == NULL)
  {
    return;
  }
  //当前控制误差角度
  bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
  //云台相对角度+ 误差角度 + 新增角度 如果大于最大机械角度
  if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
  {
    //如果是往最大机械角度控制方向
    if (add > 0.0f)
    {
      //计算出一个最大的添加角度，
      add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }
  }
  else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
  {
    if (add < 0.0f)
    {
      add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
    }
  }
  angle_set = gimbal_motor->absolute_angle_set;
  gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  gimbal_motor->relative_angle_set += add;
  //是否超过最大 最小值
  if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
  }
  else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
  {
    gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
  }
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop()
{
  if (gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_control.gimbal_yaw_motor.given_current = 0;
  }
  else if(gimbal_control.gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
	gimbal_motor_relative_angle_control(&gimbal_control.gimbal_yaw_motor);
  }
  else
  {
    gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_yaw_motor);
  }



  
  if (gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
  {
    gimbal_control.gimbal_pitch_motor.given_current = 0;
  }
  else if(gimbal_control.gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
  {
	gimbal_motor_relative_angle_control(&gimbal_control.gimbal_pitch_motor);
  }
  else
  {
    gimbal_motor_absolute_angle_control(&gimbal_control.gimbal_pitch_motor);
  }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  //角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set =  gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

  //控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
  if (gimbal_motor == NULL)
  {
    return;
  }
  //角度环，速度环串级pid调试
  gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
  gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
  //控制值赋值
  gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief 云台PID初始化函数
 *
 * @param pid PID数据结构指针
 * @param kp
 * @param ki
 * @param kd
 * @param kf
 * @param maxout 最大输出
 * @param max_iout 最大积分输出
 */
 static void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float kf, float maxout, float max_iout)
{
  if (pid == NULL)
  {
    return;
  }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->kf = kf;

  pid->err = 0.0f;
  pid->get = 0.0f;

  pid->max_iout = max_iout;
  pid->max_out = maxout;
}

/**
 * @brief 云台PID计算函数
 *
 * @param pid
 * @param get
 * @param set
 * @param error_delta 误差微分项，直接从传感器读取，不计算
 * @return fp32
 */
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta)
{
  float err;
  if (pid == NULL)
  {
    return 0.0f;
  }
  pid->get = get;
  pid->set = set;

  err = set - get;
  pid->err = rad_format(err);
  pid->Pout = pid->kp * pid->err;
  pid->Iout += pid->ki * pid->err;
  pid->Dout = pid->kd * error_delta;
 // pid->Fout = pid->kf*(pid->set-pid->last_set);
  abs_limit(&pid->Iout, pid->max_iout);
  pid->out = pid->Pout + pid->Iout + pid->Dout;//+pid->Fout;
  abs_limit(&pid->out, pid->max_out);
//  if(pid->last_set!=pid->set)//前馈是用上次的set值
//  {
//	pid->last_set=pid->set;
//  }
  return pid->out;
}

/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
  if (gimbal_pid_clear == NULL)
  {
    return;
  }
  gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
  gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
  return &gimbal_control.gimbal_pitch_motor;
}
/**
 * @brief 云台姿态数据发送到上位机
 *
 */
void gimbal_data_send(void)
{
  //10*2ms,每20ms给自瞄发送一次信息
  static uint8_t send_time;
  if (send_time == 10)
  {
    //user_data_pack_handle();
    send_time = 0;
  }
  send_time++;
}
/**
 * @brief          判断上位机是否掉线
 * @param[out]     void
 * @retval         void
 */
void NUC_GG_Detector(void)
{
	NUC_GG_detect++;
	if(NUC_GG_detect >= 50)
	auto_target.transmit_good = 0,
	NUC_GG_detect = 50;
}

/**
 * @brief          如果电机安装方向相反，需要在电流值前添加负号
 * @param[out]     void
 * @retval         void
 */
static void gimbal_motor_turn(void)
{
#if YAW_OUT_TURN
    yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
    yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_OUT_TURN
    pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
    pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif
}

