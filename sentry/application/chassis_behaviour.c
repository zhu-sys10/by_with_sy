/**
 * @file chassis_behaviour.c
 * @author 何清华
 * @brief
 * @version 2.0
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024 SPR
 *
 */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "config.h"
#include "referee.h"
#include "detect_task.h"
#include "radar_task.h"
#include "INS_task.h"
//给136行用
#include "shoot_task.h"
#include "USART_receive.h"
extern shoot_control_t shoot_control;

extern uint8_t get_game_status(void);
extern RxData new_auto_chassis_data;

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

int	chassis_x = 0;
int radar_missing_check = 0;
int t = 0;
uint8_t enter_YAW_flag=1;
//extern INS_data_t INS_data;

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘小陀螺的行为状态机下，底盘模式是一边旋转一边以云台指向方向运动
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief     		底盘自动控制的行为状态机下，底盘模式是自动控制，由上位机进行控制
 * @param[in] 		chassis_move_rc_to_vector底盘数据
 * @retval    		返回空
 */
static void Chassis_IPC_Control(chassis_move_t *chassis_move_rc_to_vector);
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
 
static void Chassis_Game_Control(chassis_move_t *chassis_move_rc_to_vector);
/**
 * @brief          比赛时底盘模式
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
 
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);
/**
 * @brief          底盘模式选择
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
extern uint8_t radar_flag_number;
extern chassis_move_t chassis_move;//底盘运动数据

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }




  //遥控器设置模式(非必要不建议改)
	if (switch_is_up(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 左上
	{
		chassis_move_mode->chassis_mode = CHASSIS_AUTO;// CHASSIS_AUTO_GAME;
	   enter_YAW_flag=2;
    }
	else if (switch_is_mid(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 左中
	{
        chassis_move_mode->chassis_mode = CHASSIS_TOP;
		// chassis_move_mode->chassis_mode = CHASSIS_OPEN;
		// 	/*在左拨杆拨中时，旋转滚轮实现底盘停下*/
		// 	if(shoot_control.shoot_mode == SHOOT_STOP && _abs(rc_ctrl.rc.ch[RC_CIRCLE]) > 0)
		// 	{
		// 		chassis_move_mode->chassis_mode = CHASSIS_TOP;
		// 	}
	  enter_YAW_flag=0;
	}
	else if (switch_is_down(rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL])) // 左下
	{
		chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
		enter_YAW_flag=0;
			// /*在左下角拨杆时，旋转滚轮实现小陀螺旋转，滚轮越偏小陀螺转速越快*/
			// if(shoot_control.shoot_mode == SHOOT_STOP && _abs(rc_ctrl.rc.ch[RC_CIRCLE]) > 0)
			// {
			// 	chassis_move_mode->chassis_mode = CHASSIS_TOP;
			// }
	}

  //当云台在某些模式下，像初始化， 底盘不动
  if (gimbal_cmd_to_chassis_stop())
  {
    chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE;
  }
  
}


/**
 * @brief          设置控制量.根据不同底盘控制模式，会调用不同的控制函数.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
  {
    chassis_no_move_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
  {
    chassis_follow_gimbal_yaw_control(chassis_move_rc_to_vector);
  }
	else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_TOP)
  {
    chassis_top_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_GIMBAL)
  {
    chassis_no_follow_yaw_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_OPEN)
  {
    chassis_open_set_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
  {
    chassis_zero_force_control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_AUTO)
  {
	Chassis_IPC_Control(chassis_move_rc_to_vector);
  }
  else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_AUTO_GAME)
  {
	Chassis_Game_Control(chassis_move_rc_to_vector);
  }
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_follow_gimbal_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  { 
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f;

  //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
  chassis_rc_to_control_vector(&vy_set, &vx_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  chassis_move_rc_to_vector->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
  chassis_move_rc_to_vector->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

  //设置控制相对云台角度
  chassis_move_rc_to_vector->chassis_relative_angle_set = 0;
  chassis_move_rc_to_vector->chassis_relative_angle = rad_format(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  
  // 速度限幅
  chassis_move_rc_to_vector->vx_set = -float_constrain(chassis_move_rc_to_vector->vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vy_set = -float_constrain(chassis_move_rc_to_vector->vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
  
  // 计算旋转PID角速度
  chassis_move_rc_to_vector->wz_set = -PID_calc(&chassis_move_rc_to_vector->chassis_angle_pid, chassis_move_rc_to_vector->chassis_relative_angle, chassis_move_rc_to_vector->chassis_relative_angle_set);
}

/**
 * @brief          底盘小陀螺的行为状态机下，底盘模式是一边旋转一边以云台指向方向运动
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_top_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f;

  //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
  chassis_rc_to_control_vector(&vy_set, &vx_set, chassis_move_rc_to_vector);
  float sin_yaw = 0.0f, cos_yaw = 0.0f;
  //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);

  chassis_move_rc_to_vector->vx_set = -(cos_yaw * vx_set - sin_yaw * vy_set);//方向
  chassis_move_rc_to_vector->vy_set = -(sin_yaw * vx_set + cos_yaw * vy_set);

  //设置小陀螺转速
  if (vx_set == 0 & vy_set == 0)
  {
    chassis_move_rc_to_vector->wz_set = CHASSIS_TOP_SPEED*(rc_ctrl.rc.ch[RC_CIRCLE])/(RC_CH_VALUE_MAX-RC_CH_VALUE_OFFSET);
  }
  else
  {
    //平移时小陀螺速度变慢
    chassis_move_rc_to_vector->wz_set = 1.0f * CHASSIS_TOP_SPEED*(rc_ctrl.rc.ch[RC_CIRCLE])/(RC_CH_VALUE_MAX-RC_CH_VALUE_OFFSET);
  }
  
}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

  chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_rc_to_vector);
  wz_set = -CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL];

  chassis_move_rc_to_vector->wz_set = wz_set;
  chassis_move_rc_to_vector->vy_set = -float_constrain(vx_set, chassis_move_rc_to_vector->vx_min_speed, chassis_move_rc_to_vector->vx_max_speed);
  chassis_move_rc_to_vector->vx_set = -float_constrain(vy_set, chassis_move_rc_to_vector->vy_min_speed, chassis_move_rc_to_vector->vy_max_speed);
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */
static void chassis_open_set_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_move_rc_to_vector->vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->vy_set = -rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  chassis_move_rc_to_vector->wz_set = -rc_ctrl.rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  chassis_move_rc_to_vector->vx_set = 0.0f;
  chassis_move_rc_to_vector->vy_set = 0.0f;
  chassis_move_rc_to_vector->wz_set = 0.0f;
}

/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel, vy_channel;
  float vx_set_channel, vy_set_channel;

  //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(rc_ctrl.rc.ch[RC_RIGHT_X], vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(rc_ctrl.rc.ch[RC_RIGHT_Y], vy_channel, CHASSIS_RC_DEADLINE);
  vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
  vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
  }

  *vx_set = vx_set_channel;
  *vy_set = vy_set_channel;
}
/**
 * @brief     		底盘自动控制的行为状态机下，底盘模式是自动控制，由上位机进行控制
 * @param[in] 		chassis_move_rc_to_vector底盘数据
 * @retval    		返回空
 */
static void Chassis_IPC_Control(chassis_move_t *chassis_move_rc_to_vector)
{
	//直接用雷达发来的数据
	if(new_auto_chassis_data.linear_x!=0 || new_auto_chassis_data.linear_y!=0)
	{
		chassis_move_rc_to_vector->wz_set = 0.0f;// *1.1
		chassis_move_rc_to_vector->vx_set = new_auto_chassis_data.linear_x;
		chassis_move_rc_to_vector->vy_set = new_auto_chassis_data.linear_y;
		auto_chassis.new_chassis_data_flag = 0;
	}
	else
	{
		chassis_move_rc_to_vector->wz_set = 0.0f;
		chassis_move_rc_to_vector->vx_set = 0.0f;
		chassis_move_rc_to_vector->vy_set = 0.0f;
		auto_chassis.new_chassis_data_flag = 0;
	}
		
	
    //计算正对时刻的坐标系和底盘姿态角之间的差值
    //如果云台一直旋转，旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
	float sin_yaw = 0.0f, cos_yaw = 0.0f;
    //yaw_relative——angle=0.000766
    float vx_set = 0.0f, vy_set = 0.0f;
	//旋转矩阵
    sin_yaw = arm_sin_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
	cos_yaw = arm_cos_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
    vx_set = chassis_move_rc_to_vector->vx_set;
    vy_set = chassis_move_rc_to_vector->vy_set;
    
    chassis_move_rc_to_vector->vx_set = cos_yaw * vx_set - sin_yaw *vy_set;
	chassis_move_rc_to_vector->vy_set = sin_yaw * vx_set + cos_yaw *vy_set;

    if(toe_is_error(DBUS_TOE))
	{
		chassis_move_rc_to_vector->wz_set = 0.0f;
		chassis_move_rc_to_vector->vx_set = 0.0f;
		chassis_move_rc_to_vector->vy_set = 0.0f;
	}
}

//待比赛前修改
static void Chassis_Game_Control(chassis_move_t *chassis_move_rc_to_vector)
{	


    float vx_set = 0.0f, vy_set = 0.0f;
	if(auto_chassis.vx_set!=0 || auto_chassis.vy_set!=0)
	{
		vx_set = (auto_chassis.vx_set);// /2.3
		vy_set = (auto_chassis.vy_set);
	}
	else
	{
		vx_set = 0.0f;
		vy_set = 0.0f;
	}  

  float sin_yaw = 0.0f, cos_yaw = 0.0f;
	
  //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
  sin_yaw = arm_sin_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  cos_yaw = arm_cos_f32(chassis_move_rc_to_vector->chassis_yaw_motor->relative_angle);
  chassis_move_rc_to_vector->vx_set = (cos_yaw * vx_set - sin_yaw * vy_set);//方向
  chassis_move_rc_to_vector->vy_set = (sin_yaw * vx_set + cos_yaw * vy_set);
	
		//wz受击打再旋转
		if(chassis_and_gimbal.chassis_start_top_flag==1)//当发生被弹丸击打时或在被弹丸击打后的一段时间内
		{			
			chassis_move_rc_to_vector->wz_set = chassis_and_gimbal.chassis_wz_speed+CHASSIS_TOP_SPEED/2;//第一次受击打相关的旋转速度+初始旋转速度
			chassis_and_gimbal.chassis_turn_time++;
				if(chassis_move_rc_to_vector->wz_set>=CHASSIS_TOP_SPEED)//wz限幅
				{
					chassis_move_rc_to_vector->wz_set = CHASSIS_TOP_SPEED;
				}
		}
		else
		{
			chassis_move_rc_to_vector->wz_set = 0;
		}
		if(chassis_and_gimbal.chassis_turn_time>=2500)//底盘旋转时间
		{
			chassis_and_gimbal.chassis_start_top_flag=0;
			chassis_and_gimbal.chassis_turn_time=0;
			chassis_move_rc_to_vector->wz_set = 0;
			chassis_and_gimbal.chassis_wz_speed=0;
		}
	
  if(auto_chassis.wz_flag==1)
  {
	chassis_move_rc_to_vector->wz_set=auto_chassis.wz_set;
  }
  
 //#if GAME_MODE
			if(!(get_game_status()==4))//不在对战中，就一直不动
// 			  {
// 				chassis_move_rc_to_vector->wz_set = 0.0f;
// 				chassis_move_rc_to_vector->vx_set = 0.0f;
// 				chassis_move_rc_to_vector->vy_set = 0.0f;
// 			  }
// #endif
	
		if(toe_is_error(DBUS_TOE))
		{
			chassis_move_rc_to_vector->wz_set = 0.0f;
			chassis_move_rc_to_vector->vx_set = 0.0f;
			chassis_move_rc_to_vector->vy_set = 0.0f;
		}
	
}
