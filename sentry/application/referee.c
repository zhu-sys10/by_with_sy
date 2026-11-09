#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;

ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_rfid_status_t rfid_status_t;
ext_student_interactive_data_t student_interactive_data_t;
ext_robot_command_t robot_command_t;
//全局变量
char data_debug[16];

//外部接口
extern void gimbaler_data_OOD(char value);

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));

    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
	memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));
    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));
}

uint16_t referee_data_length; //检测数据包长度
void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
    case GAME_STATE_CMD_ID:
    {
        memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
    }
    break;
    case GAME_RESULT_CMD_ID:
    {
        memcpy(&game_result, frame + index, sizeof(game_result));
    }
    break;
    case GAME_ROBOT_HP_CMD_ID:
    {
        memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
    }
    break;

    case FIELD_EVENTS_CMD_ID:
    {
        memcpy(&field_event, frame + index, sizeof(field_event));
    }
    break;
    case SUPPLY_PROJECTILE_ACTION_CMD_ID:
    {
        memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
    }
    break;
    case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
    {
        memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
    }
    break;
    case REFEREE_WARNING_CMD_ID:
    {
        memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
    }
    break;

    case ROBOT_STATE_CMD_ID:
    {
        memcpy(&robot_state, frame + index, sizeof(robot_state));
    }
    break;
    case POWER_HEAT_DATA_CMD_ID:
    {
		memcpy(&referee_data_length, frame + 1, sizeof(referee_data_length));
        memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
		//memcpy(&power_heat_data_t, frame + index, 16);
		memcpy(data_debug, frame + index, 16);
    }
    break;
    case ROBOT_POS_CMD_ID:
    {
        memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
    }
    break;
    case BUFF_MUSK_CMD_ID:
    {
        memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
    }
    break;
    case AERIAL_ROBOT_ENERGY_CMD_ID:
    {
        memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
    }
    break;
    case ROBOT_HURT_CMD_ID:
    {
        memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));	
    }
    break;
    case SHOOT_DATA_CMD_ID:
    {
        memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
    }
    break;
    case BULLET_REMAINING_CMD_ID:
    {
        memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
    }
    break;
		case RFID_STATUS_CMD_ID:
		{
				memcpy(&rfid_status_t, frame + index, sizeof(ext_rfid_status_t));
		}
		break;
    case STUDENT_INTERACTIVE_DATA_CMD_ID:
    {
        memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
    }
    break;
	case STUDENT_ROBOT_COMMAND_CMD_ID:
    {
        memcpy(&robot_command_t, frame + index, sizeof(robot_command_t));
//		gimbaler_data_OOD(1);
//		Gimbaler_Data_Solve();
    }
    break;
    default:
    {
        break;
    }
    }
}

void get_chassis_power_and_buffer_and_power_limit(float *power, float *buffer, float *power_limit)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
	  *power_limit = robot_state.chassis_power_limit;
}




uint8_t get_robot_id(void) //获取当前机器人ID
{
    return robot_state.robot_id;
}

uint16_t get_robot_HP(void)//获取当前机器人HP
{
	return robot_state.remain_HP;
}

uint8_t get_game_status(void) //获取比赛状况, 0：未开始比赛；1：准备阶段；2：自检阶段；3：5s 倒计时；4：对战中；5：比赛结算中
{
	return game_state.game_progress;
}

uint16_t get_game_stage_remain_time(void) //获取比赛时间
{
	return game_state.stage_remain_time;
}


void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat0, uint16_t *heat1) //枪管热量上限以及热量
{
    *heat_limit = robot_state.shooter_17mm_cooling_limit;
    *heat0 = power_heat_data_t.shooter_heat0;
    *heat1 = power_heat_data_t.shooter_heat1;
}

void get_shoot_speed(float *speed1,float *speed2){
		if(shoot_data_t.barrel_ID == 1)
		{
			*speed1 = shoot_data_t.bullet_speed;
		}
		else if(shoot_data_t.barrel_ID == 2)
		{
			*speed2 = shoot_data_t.bullet_speed;
		}
		return;
}

uint16_t get_shoot_heat0(void)
{
	return power_heat_data_t.shooter_heat0;
}

uint16_t get_shoot_heat1(void)
{
	return power_heat_data_t.shooter_heat1;
}

unsigned char get_hurt_type(void) //获取伤害类型
{
	return robot_hurt_t.hurt_type;
}

unsigned char get_hurted_armer(void) //获取收到伤害的装甲ID
{//当前车：0正前方，1左边，2后面，3 右边
	return robot_hurt_t.armor_type;
}

unsigned char Get_Team(void) //获取当前队伍。0为红方，1为蓝方
{
	if(robot_state.robot_id <= 9 && robot_state.robot_id >= 1)
		return 0;
	return 1;
}

uint16_t Get_Outpost_HP(char team) //获取前哨站血量。参数一：队伍。0为红方，1为蓝方。
{
	if(team)
		return game_robot_HP_t.blue_outpost_HP;//蓝方
	else
		return game_robot_HP_t.red_outpost_HP;//红方
}

/**
 * 获取当前剩余子弹数量的函数
 * 
 * @return 返回当前剩余的子弹数量，类型为uint16_t
 */
uint16_t get_bullet_allowance(void)
{
	return bullet_remaining_t.bullet_remaining_num;
}

/**
 * 获取哨兵巡逻区域状态
 * @return 返回哨兵巡逻区域状态，1表示激活，0表示未激活
 */
uint32_t get_sentry_patrol_zone_status(void)
{
	// 通过位操作获取rfid_status的第14位状态
	return ((rfid_status_t.rfid_status >> 14) & 1);
}
