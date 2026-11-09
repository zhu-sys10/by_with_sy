/**
 * @file     shoot_task.c
 * @author   何清华，lkhannn
 * @brief    上下双枪哨兵的摩擦轮、拨盘控制
 * @version  2.0
 * @date     2024-10-28
 * @copyright Copyright (c) 2024 SPR
 *
 */
#include "shoot_task.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "USART_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "referee.h"
#include "chassis_behaviour.h"
#include "radar_task.h"
#include "config.h"
#include "user_lib.h"

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义

extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_supply_projectile_booking_t supply_projectile_booking_t;
extern uint8_t get_game_status(void);

uint8_t move_flag = 0;//拨盘单发判断标志位，置1时拨盘单发
uint16_t speed_cut=0;
pid_type_def motor_barrel_pid;
shoot_control_t shoot_control;

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_init(void);

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
static void shoot_control_loop(void);

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void);

/**
 * @brief          堵转倒转处理
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_turn_back(void);

void fire_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(SHOOT_TASK_INIT_TIME);
  //发射机构初始化
  shoot_init();
  //判断摩擦轮电机是否都在线
//   while (toe_is_error(LEFT_FRICTION_MOTOR_TOE) || toe_is_error(RIGHT_FRICTION_MOTOR_TOE) )
//   {
//    vTaskDelay(SHOOT_CONTROL_TIME);
//   }
  while (1)
  {

    // ① 反馈
    shoot_control.trigger_motor.speed = shoot_control.trigger_motor.motor_measure->speed_rpm;

    // ② 目标(先小一点)
    shoot_control.trigger_motor.speed_set = 1000.0f;

    // ③ PID
    PID_calc(&shoot_control.trigger_motor.motor_speed_pid,
             shoot_control.trigger_motor.speed,
             shoot_control.trigger_motor.speed_set);

    // ④ 发送
    CAN_cmd_shoot((int16_t)shoot_control.trigger_motor.motor_speed_pid.out);

    vTaskDelay(5);
}
//     //射击模式选择
//      shoot_set_mode();
//     shoot_feedback_update();
//     //射击控制量计算
//     shoot_control_loop();
// 	//电流设置  
//    if (!(toe_is_error(LEFT_FRICTION_MOTOR_TOE) || toe_is_error(RIGHT_FRICTION_MOTOR_TOE)))
//    {
		//正号顺时针，负号逆时针
		// CAN_cmd_shoot(500);
        //  vTaskDelay(5); 
// 		CAN_cmd_fricion(shoot_control.left_friction.given_current, shoot_control.right_friction.given_current);
//   }

//     vTaskDelay(SHOOT_CONTROL_TIME);
  }

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
  shoot_control.shoot_mode = SHOOT_STOP;
  //电机指针
  shoot_control.trigger_motor.motor_measure = get_trigger_motor_measure_point();
  //初始化PID
  PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT);
  PID_init(&shoot_control.left_friction.motor_speed_pid, UP_FRICTION_KP, UP_FRICTION_KI, UP_FRICTION_KD, UP_FRICTION_MAX_OUT, UP_FRICTION_MAX_IOUT); //左摩擦轮
  PID_init(&shoot_control.right_friction.motor_speed_pid, UP_FRICTION_KP, UP_FRICTION_KI, UP_FRICTION_KD, UP_FRICTION_MAX_OUT, UP_FRICTION_MAX_IOUT); //右摩擦轮

  //更新数据
  shoot_feedback_update();
  //拨弹电机
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;
  shoot_control.trigger_motor.given_current = 0;
  shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
  shoot_control.trigger_motor.speed = 0.0f;
  shoot_control.trigger_motor.speed_set = 0.0f;
}

/**
 * @brief          射击状态机设置，遥控器上拨1次发射数颗，一直处在上，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */

void AutoShootTask(shoot_mode_e* shoot_mode)
{	
	if(auto_target.transmit_good)
	{
		if(auto_target.fire == 1 )
		{
			//连发
			//*shoot_mode = SHOOT_BULLET;
			//单发
			shoot_bullet_control();
		}
		else
		{
			*shoot_mode = SHOOT_READY;
		}
	}
	else
	{
		*shoot_mode = SHOOT_READY;
	}
	if (shoot_control.trigger_motor.angle - shoot_control.trigger_motor.set_angle > 0)
	{
		shoot_control.shoot_mode = SHOOT_SINGLE_BULLET;
	}
#if GAME_MODE
				if(!(get_game_status()==4))//不在对战中，就一直不射
				{
					*shoot_mode = SHOOT_READY;
				}
#endif
}

static void shoot_set_mode(void)
{
	/*左拨上表示比赛模式*/
	if(switch_is_up(rc_ctrl.rc.s[LEFT_SWITCH])) // 左拨杆上
	{
		//右拨杆拨到下时，不会发射
		if(!switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) )
		{
			AutoShootTask(&shoot_control.shoot_mode);
		}
		else
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		 
		//摩擦轮速度没到位时会预热摩擦轮，防止开机锁定导致摩擦轮不动
		if( (shoot_control.left_friction.speed_set - 500 > shoot_control.left_friction.speed) ||  shoot_control.left_friction.speed_set == 0)
			shoot_control.shoot_mode = SHOOT_READY;
	
	}
		/*左拨杆中，自瞄调试*/
	else if(switch_is_mid(rc_ctrl.rc.s[LEFT_SWITCH]))
	{
		if(!switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]) )
		{
			AutoShootTask(&shoot_control.shoot_mode);
		}
		else
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
	}
		/*左拨杆下，正常模式*/
	else if(switch_is_down(rc_ctrl.rc.s[LEFT_SWITCH]))
	{
		if (switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
		{
			shoot_control.shoot_mode = SHOOT_BULLET;
		}
		else if (switch_is_mid(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
		{
			shoot_control.shoot_mode = SHOOT_READY;
				
					//遥控器左边那个圆滚，直控单发
					static char gun_switch = 1;
					if(_abs(rc_ctrl.rc.ch[RC_CIRCLE]) > 200)
					{
						if(gun_switch == 0)
						{
							shoot_bullet_control();
							gun_switch = 1;
						}
					}
					else
					{
						gun_switch = 0;
					}
					if (shoot_control.trigger_motor.angle - shoot_control.trigger_motor.set_angle < 0)
					{
						shoot_control.shoot_mode = SHOOT_SINGLE_BULLET;
					}
			
		}
		else if (switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		
	}
#if GAME_MODE		
	
	if((get_game_status()==4))//不在对战中，就不热量限制，不降射速
	{
		//热量限制 下枪管1号，仅能读到1号
		get_shoot_heat_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat0, &shoot_control.heat1);
		if(power_heat_data_t.shooter_heat0 >= shoot_control.heat_limit-60)
		{
			shoot_control.shoot_mode = SHOOT_READY;
		}
		if(shoot_control.shoot_speed1>=25.0f)//如果超射速就降低射速
		{
			speed_cut=300;
		}				
	}

#endif
	
	//如果云台状态是无力状态或者枪管电机没有就位，就关闭射击
	// if (gimbal_cmd_to_shoot_stop())
	//   shoot_control.shoot_mode = SHOOT_READY;

	//枪保险（必须放最后）
	if(switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
		shoot_control.shoot_mode = SHOOT_STOP;

}

/**
 * @brief 射击数据更新
 *@param[in] void
 *@retval void
 */
static void shoot_feedback_update(void)
{
  get_shoot_speed(&shoot_control.shoot_speed1,&shoot_control.shoot_speed2);
	
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;

  shoot_control.trigger_motor.speed = shoot_control.trigger_motor.motor_measure->speed_rpm;
  //左摩擦轮
  shoot_control.left_friction.speed = shoot_control.left_friction.motor_measure.speed_rpm;
  //右摩擦轮
  shoot_control.right_friction.speed = shoot_control.right_friction.motor_measure.speed_rpm;

}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */

static void shoot_control_loop(void)
{
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    //激光关闭
    shoot_laser_off();
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0;
	shoot_control.left_friction.speed_set = 0;
	shoot_control.right_friction.speed_set = 0;
    //设置加速减速电流限制
    shoot_control.left_friction.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
    shoot_control.right_friction.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    if (shoot_control.right_friction.speed > (FRICTION_SPEED_SET * 0.5f))
    {
      shoot_control.left_friction.motor_speed_pid.max_out = FRICTION_MAX_OUT;
      shoot_control.right_friction.motor_speed_pid.max_out = FRICTION_MAX_OUT;
    }
      shoot_control.left_friction.speed_set = -FRICTION_SPEED_SET-speed_cut;
      shoot_control.right_friction.speed_set = FRICTION_SPEED_SET+speed_cut;
	  shoot_control.trigger_motor.speed_set = 0.0f;
  }
  else if (shoot_control.shoot_mode == SHOOT_BULLET)
  {
	//设置拨弹轮的拨动速度
	shoot_control.trigger_motor.speed_set = -TRIGGER_SPEED;	 
	//堵转反转
    trigger_motor_turn_back();
  }
  else if(shoot_control.shoot_mode == SHOOT_SINGLE_BULLET)
  {
		if (shoot_control.right_friction.speed > (FRICTION_SPEED_SET * 0.5f))
		{
		  shoot_control.left_friction.motor_speed_pid.max_out = FRICTION_MAX_OUT;
		  shoot_control.right_friction.motor_speed_pid.max_out = FRICTION_MAX_OUT;
		}
		  shoot_control.left_friction.speed_set = -FRICTION_SPEED_SET-speed_cut;
		  shoot_control.right_friction.speed_set = FRICTION_SPEED_SET+speed_cut;
		  shoot_control.trigger_motor.speed_set = 0.0f;
	  
		if (shoot_control.trigger_motor.angle - shoot_control.trigger_motor.set_angle < 0)
		{
			//没到达一直设置旋转速度
			shoot_control.trigger_motor.speed_set = -AUTO_TRIGGER_SPEED;
			//开启反堵转处理
			trigger_motor_turn_back();
		}
		else
		{
			shoot_control.trigger_motor.speed_set = 0;
		}
  }

			// if (shoot_control.shoot_mode != SHOOT_STOP)
			// {
			// shoot_laser_on(); //激光开启(应该是控制c板的那个5V，我们没用到)
			// }
	
    //遥控器下电，摩擦轮拨盘速度置零，直接电流置0，有个摩擦轮会疯转
	if (toe_is_error(DBUS_TOE))
	{
	shoot_control.trigger_motor.speed_set = 0;
	shoot_control.left_friction.speed_set = 0;
	shoot_control.right_friction.speed_set = 0;
	}
	
	//计算拨弹轮电机PID
	PID_calc(&shoot_control.trigger_motor.motor_speed_pid, shoot_control.trigger_motor.speed, shoot_control.trigger_motor.speed_set);
	shoot_control.trigger_motor.given_current = (int16_t)(shoot_control.trigger_motor.motor_speed_pid.out);

	//计算摩擦轮PID
	PID_calc(&shoot_control.left_friction.motor_speed_pid, shoot_control.left_friction.motor_measure.speed_rpm, shoot_control.left_friction.speed_set);
	shoot_control.left_friction.given_current = (int16_t)(shoot_control.left_friction.motor_speed_pid.out);
	PID_calc(&shoot_control.right_friction.motor_speed_pid, shoot_control.right_friction.motor_measure.speed_rpm, shoot_control.right_friction.speed_set);
	shoot_control.right_friction.given_current = (int16_t)(shoot_control.right_friction.motor_speed_pid.out);
}

/**
 * @brief 卡弹检测及反转处理
 *
 */
static void trigger_motor_turn_back(void)
{
  if (shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.trigger_motor.speed_set = shoot_control.trigger_motor.speed_set;
  }
  else
  {
    shoot_control.trigger_motor.speed_set = -shoot_control.trigger_motor.speed_set;
  }

  if (fabs(shoot_control.trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
  {
    shoot_control.block_time++;
    shoot_control.reverse_time = 0;
  }
  else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
  {
    shoot_control.reverse_time++;
  }
  else
  {
    shoot_control.block_time = 0;
  }
}

/**
 * @brief          射击控制，控制拨弹电机角度，完成一次发射
 * @param[in]      void
 * @retval         void
 */
static void shoot_bullet_control(void)
{
    shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle +2275; //3000.0f;//2500.0f,8192/10是一颗子弹的
	shoot_control.shoot_mode = SHOOT_SINGLE_BULLET;
}
