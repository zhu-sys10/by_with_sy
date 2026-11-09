//哨兵比赛时的行为状态机，明年大概率用不到，只是暂时保留

/**
* @file radar_task.c
* @author Lkhannn
* @brief navigation
* @version 1.0
* @date 2024-04-24
*
* @copyright Copyright (c) 2024 SPR
*
*/
#include "radar_task.h"
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
#include "fsm.h"
#include "CAN_receive.h"
#include "shoot_task.h"

#define pointA Ring_Shaped_Elevated_Ground
#define attack_mode 1//1-outpost 2-robot
FSM location_fsm;

int current_event = 0;
int des = 0;

extern int target_lost_time;

radar_control_t radar_control;
int test_flag = 0;

inline float f32_abs(float a)
{
	if(a >= 0) return a;
	else return -a;
}

/**
 * 目的地枚举类型定义
 * 该枚举用于表示不同的目的地区域，每个枚举值代表一个特定的区域类型
 */
enum destination{
 Born_Place = 1,         // 出生地，起始位置
 Restoration_Zone,
 Road_Zone,              // 道路区域
 Outpost_Zone,           // 前哨区域
 Ring_Shaped_Elevated_Ground,  // 环形高地
	Trapezoid_shaped_Elevated_Ground,
};

// 枚举类型：location_event，用于定义位置相关的事件类型
enum location_event{
start = 1,           // 开始事件
arrive_road,        // 到达道路事件
outpost_HP_low,     // 前哨站生命值低事件
outpost_HP_high,    // 前哨站生命值高事件
arrive_born,        // 到出生点事件
arrive_ring,        // 到达环形区域事件
outpost_time_dead,  // 前哨站超时死亡事件
under_attack,       // 受到攻击事件
low_HP,            // 生命值低事件
no_attack,         // 无攻击事件
arrive_re,         // 到达特定区域事件
re_finish,         // 区域任务完成事件
out_patrol,        // 出去巡逻事件
face_centre_road,  // 面向中心道路事件
face_centre_born,  // 面向出生点事件
radar_dead,        // 雷达失效事件
try_again,         // 重试事件
all_over,          // 全部结束事件
};

/**
 * @brief 位置状态枚举
 * 
 * 该枚举定义了系统可能存在的各种位置状态，每个状态代表系统在特定环境下的运行状态
 */
enum location_state
{
	OFF = 1,       // 关闭状态，表示系统处于关闭状态
 NAV_ROAD,      // 导航道路状态，表示系统正在道路上导航
 WAIT_ROAD,     // 等待道路状态，表示系统正在等待道路条件
 NAV_BORN,      // 导航出生点状态，表示系统正在导航到出生点
 NAV_RING,      // 导航环状区域状态，表示系统正在导航到环状区域
 SPIN_WAIT,     // 旋转等待状态，表示系统正在旋转并等待
 WAIT_RING,     // 等待环状区域状态，表示系统正在等待环状区域条件
 CRAZY_SPIN,    // 疯狂旋转状态，表示系统处于快速旋转状态
 NAV_RE,
 WAIT_HP,       // 等待高精度状态，表示系统正在等待高精度定位
	BACK_CENTRE,   // 返回中心状态，表示系统正在返回到中心位置
	WAIT_BORN,
};


static void decision_data_update(void);
static void HP_deduction_check(uint16_t current_HP);
static void sentry_patrol_check(uint32_t patrol_status);
static void des_change_check(void);

void off(void);
void nav_road(void);	
void wait_road(void);
void nav_born(void);
void nav_ring(void);
void wait_ring(void);
void spin_wait(void);
void crazy_spin(void);
void nav_re(void);
void wait_hp(void);
void back_centre(void);
void wait_born(void);

FsmTable location_table[] =
{
		{OFF, start, NAV_ROAD, nav_road},
		{NAV_ROAD,radar_dead,NAV_BORN,nav_born},
   {NAV_ROAD,arrive_road,WAIT_ROAD,wait_road},
		{WAIT_ROAD,outpost_HP_low,NAV_BORN,nav_born},
   {NAV_BORN,arrive_born,WAIT_BORN,wait_born},
		{WAIT_BORN,try_again,NAV_ROAD,nav_road},
		{WAIT_BORN,all_over,SPIN_WAIT,spin_wait},
		
};



/**
 * @brief 雷达任务函数，负责雷达的状态管理和控制
 * @param pv任务参数，此处未使用
 */
void radar_task(void const *pvParameters)
{
    // 初始延时，等待系统初始化完成
 vTaskDelay(RADAR_TASK_INIT_TIME);

    // 注册状态机
 FSM_Regist(&location_fsm, location_table);
    // 设置初始状态为OFF
	location_fsm.curState = OFF;
    // 计算状态表大小
	location_fsm.size = sizeof(location_table) / sizeof(FsmTable);
    // 清空状态转移标志
	location_fsm.transfer_flag = 0;
    // 设置初始事件为start
	current_event = start;
	
    // 更新决策数据
	decision_data_update();
 
    // 主循环
 while (1)
 {
        // 检查测试标志是否为1
   if(test_flag == 1)//(switch_is_up(rc_ctrl.rc.s[GIMBAL_MODE_CHANNEL])) //left_up
   {
            // 处理当前事件
     FSM_EventHandle(&location_fsm, current_event);
   }
   else
   {
            // 如果测试标志不为1，重置状态和参数
     location_fsm.curState = OFF;
     current_event = start;
            // 清空旋转标志
			radar_control.spin_flag = 0;
			radar_control.crazy_spin_flag = 0;
            // 重置目标距离
			des = 0;
   }
		
        // 更新决策数据
		decision_data_update();
	
		//send referee data to radar
		//CAN_cmd_navigation(des);

   vTaskDelay(RADAR_CONTROL_TIME);
 }
}

/**
 * @brief 更新决策所需的数据信息
 * 该函数用于获取和更新当前游戏状态的各种关键数据，
 * 包括基地血量、视野丢失时间、自身血量、巡逻状态和弹药余量等
 */
static void decision_data_update(void)
{
    // 获取我方基地的血量值
	radar_control.my_outpost = Get_Outpost_HP(Get_Team());
    // 获取敌方基地的血量值
	radar_control.enemy_outpost = Get_Outpost_HP(!(Get_Team()));
    // 更新目标丢失时间
	radar_control.vision_lost_time = target_lost_time;
    // 获取当前剩余游戏时间（已被注释）
	//radar_control.game_remain_time = get_game_stage_remain_time();
    // 获取当前机器人的血量值
	radar_control.my_HP = get_robot_HP();
    // 获取当前巡逻区域的状态
	radar_control.patrol_status = get_sentry_patrol_zone_status();
    // 获取当前剩余弹药数量
	radar_control.bullet_allowance = get_bullet_allowance();
    // 检查血量变化情况
	HP_deduction_check(radar_control.my_HP);
    // 检查巡逻状态
	sentry_patrol_check(radar_control.patrol_status);
}

static void HP_deduction_check(uint16_t current_HP)
{
 static int hurt_time = 0;
	if(current_HP < radar_control.previous_HP)
 {
   radar_control.HP_deduction_flag = 1;
   hurt_time = 0;
 }
 else
 {
   hurt_time++;
   if(hurt_time > 3000)
   {
     radar_control.HP_deduction_flag = 0;
   }
 }
	radar_control.previous_HP = current_HP;
}

static void sentry_patrol_check(uint32_t patrol_status)
{
	//static int update_flag = 0;
	if(patrol_status == 1)
	{
		radar_control.leave_time = radar_control.game_remain_time;
	}
}


/**
 * @brief 检查是否需要改变目标位置
 * @details 该函数用于在自动底盘模式下，检查持续进行某项操作的时间是否达到阈值，
 *          如果达到阈值则设置改变目标位置的标志位
 */
static void des_change_check(void)
{
    // 静态变量，用于记录持续进行操作的时间
	static int ongoing_duration = 0; 
    // 静态变量，用于记录上一次调用时剩余的时间
	static int last_time = 0; 
	
    // 检查自动底盘是否正在进行
	if(auto_chassis.ongoing_flag == 1) {
        // 如果不是第一次调用，计算时间差并累加到持续时间中
		if(last_time != 0)
		{
			ongoing_duration += (last_time - radar_control.game_remain_time);
		}

        // 更新上一次的时间为当前剩余时间
		last_time = radar_control.game_remain_time;

        // 检查持续时间是否达到30秒
		if(ongoing_duration >= 30) 
		{
            // 重置持续时间
			ongoing_duration = 0;
            // 设置改变目标位置的标志位为1
			radar_control.change_des_flag = 1; 
			return;
		}
	}
	else
	{
        // 如果自动底盘未在进行，重置持续时间和上一次时间
		ongoing_duration = 0;
		last_time = 0;
	}
	
    // 设置改变目标位置的标志位为2
	radar_control.change_des_flag = 2; 
	return;
}


void off(void)
{
 des = 0;
 return;
}


/**
 * @brief 导航到道路区域的状态处理函数
 * 该函数处理导航到道路区域的相关逻辑，包括雷达控制、目的地检查和状态转换等
 */
void nav_road(void)
{
    // 设置目标区域为道路区域
 des = Road_Zone;

    // 启用雷达区域搜索功能
	radar_control.region_search_flag = 1;
    // 检查弹药余量是否足够
	if(radar_control.bullet_allowance > 300)
	{
        // 如果弹药充足，设置射击前哨站标志
		radar_control.shoot_outpost_flag = 1;
	}
	else
	{
        // 如果弹药不足，清除射击前哨站标志
		radar_control.shoot_outpost_flag = 0;
	}

    // 检查是否需要更改目的地
	des_change_check();
	
    // 检查是否正在执行导航任务且目标为道路区域
	if(auto_chassis.ongoing_flag == 2 && auto_chassis.destination == Road_Zone)
	{
        // 设置当前事件为到达道路区域
		current_event = arrive_road;
        // 触发状态机转移
		location_fsm.transfer_flag = 1;
		return;
	}
	
    // 检查是否需要更改目的地
	if(radar_control.change_des_flag == 1)
	{
        // 设置当前事件为雷达目标丢失
		current_event = radar_dead;
        // 清除相关控制标志
		radar_control.shoot_outpost_flag = 0;
		radar_control.region_search_flag = 0;
        // 触发状态机转移
		location_fsm.transfer_flag = 1;
        // 更改目的地标志
		radar_control.change_des_flag = 2; 

		return;
		
	}
	
 return;
}


void wait_road(void)
{
	des = Road_Zone;
	
	radar_control.region_search_flag = 1;
	if(radar_control.bullet_allowance > 300)
	{
		radar_control.shoot_outpost_flag = 1;
	}
	else
	{
		radar_control.shoot_outpost_flag = 0;
	}

	if(radar_control.my_outpost <= 300 || radar_control.game_remain_time < 240)
 {
		current_event = outpost_HP_low;
		location_fsm.transfer_flag = 1;
		radar_control.shoot_outpost_flag = 0;
		radar_control.region_search_flag = 0;
		return;
	}
	if(radar_control.enemy_outpost == 0)
	{
		if(radar_control.my_outpost < 600)
   {
     current_event = outpost_HP_low;
			radar_control.shoot_outpost_flag = 0;
			radar_control.region_search_flag = 0;
			location_fsm.transfer_flag = 1;
   }
		
	}
 return;
}

void nav_ring(void)
{
 des = Ring_Shaped_Elevated_Ground;

	if(auto_chassis.ongoing_flag == 2 && auto_chassis.destination == Ring_Shaped_Elevated_Ground)
	{
		current_event = arrive_ring;
		location_fsm.transfer_flag = 1;
		return;
	}

	if(radar_control.my_outpost < 200 || radar_control.game_remain_time < 240)
	{
		if(auto_chassis.ramp_flag == 2)
		{
			current_event = outpost_time_dead;
			location_fsm.transfer_flag = 1;
			return;
		}
	}
		
 return;

}

void wait_ring(void)
{
 des = Ring_Shaped_Elevated_Ground;
	
	if(radar_control.my_outpost < 200 || radar_control.game_remain_time < 240)
	{
		if(auto_chassis.ramp_flag == 2)
		{
			current_event = outpost_time_dead;
			location_fsm.transfer_flag = 1;
			return;
		}
	}
}


void nav_born(void)
{
 des = Born_Place;
 
 if(auto_chassis.ongoing_flag == 2 && auto_chassis.destination == Born_Place)
	{
		current_event = arrive_born;
		location_fsm.transfer_flag = 1;
		return;
	}
	
 return;
}
		
		
void back_centre(void)
{
	static int back_flag = 0;
	radar_control.back_centre_flag = 1;
	if(f32_abs(gimbal_control.gimbal_yaw_motor.absolute_angle)<0.2f)
	{
		back_flag++;
		if(back_flag == 1000)
		{
			back_flag = 0;
			radar_control.back_centre_flag = 0;
			if(des == Road_Zone)
			{
				current_event = face_centre_road;
			}
			else
			{
				current_event = face_centre_born;
     }
			location_fsm.transfer_flag = 1;
		}
	}
	
	return;
}


void wait_born(void)
{
	if(radar_control.my_outpost > 600 && radar_control.game_remain_time > 330)
	{
		current_event = try_again;
	}
	else
	{
		current_event = all_over;
	}
	location_fsm.transfer_flag = 1;
}

		
void spin_wait(void)
{
 radar_control.spin_flag = 1;
}







void crazy_spin()
{
 radar_control.crazy_spin_flag = 1;
	
	if(radar_control.my_HP < 180)
 {
		radar_control.crazy_spin_flag = 0;
   current_event = low_HP;
   location_fsm.transfer_flag = 1;
   return;
 }
	if(radar_control.leave_time - radar_control.game_remain_time>25)
	{
		radar_control.crazy_spin_flag = 0;
   current_event = out_patrol;
   location_fsm.transfer_flag = 1;
		return;
	}
 if(radar_control.HP_deduction_flag == 0)
 {
		radar_control.crazy_spin_flag = 0;
		radar_control.spin_flag = 1;
   current_event = no_attack;
   location_fsm.transfer_flag = 1;
   return;
 }

}
		

void nav_re(void)
{
 des = Restoration_Zone;
 
 if(auto_chassis.ongoing_flag == 2 && auto_chassis.destination == Restoration_Zone)
	{
		current_event = arrive_re;
		location_fsm.transfer_flag = 1;
		return;
	}
	if(radar_control.leave_time - radar_control.game_remain_time>25)
	{
		current_event = out_patrol;
		location_fsm.transfer_flag = 1;
		return;
	}
	
 return;

}


void wait_hp(void)
{
 if(radar_control.my_HP==400)
 {
   current_event = re_finish;
   location_fsm.transfer_flag = 1;
   return;
 }
 if(radar_control.leave_time - radar_control.game_remain_time>25)
 {
   current_event = re_finish;
   location_fsm.transfer_flag = 1;
   return;
 }
}


