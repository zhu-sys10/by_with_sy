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

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "main.h"
#include "referee.h"
#include "gimbal_task.h"
#include "config.h"
#include "USART_receive.h"
#include "by_with_sy.h"

#define get_motor_measure(ptr, data)                               \
{                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static motor_measure_t motor_chassis[4];  //底盘电机
static motor_measure_t motor_gimbal[3];   //云台电机
motor_measure_t motor_trigger;     //拨弹电机
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef radar_tx_message;
static uint8_t radar_can_send_data[8];
static  uint8_t Tx_Message[8];
auto_chassis_t auto_chassis = {0.0f,0.0f,0,0,0,0}; //底盘数据
game_mesage_t game_message;
extern gimbal_control_t gimbal_control;//云台控制所有相关数据
extern uint8_t tx_buffer[26];

/**
 * @brief data of capacity
 * @param PowerData[0] input voltage
 * @param PowerData[1] cap voltage 上限电压
 * @param PowerData[2] input current
 * @param PowerData[3] target power
 */
float PowerData[4];

/**
 * @brief           计算3508电机累计旋转角度
 * @param[out]      motor:电机结构数据指针
 */
void Calc_motor_Angle(motor_measure_t *motor)
{
  if (motor->ecd - motor->last_ecd > 4095.5)
  {
    motor->round--;
  }
  else if (motor->ecd - motor->last_ecd < -4095.5)
  {
    motor->round++;
  }
  motor->angle = (motor->round * ANGLE_T + motor->last_ecd) * 0.04395067757f; //转换为360度一圈
}


uint16_t uint16_t_abs(uint16_t a)
{
	if(a <= 0) return -a;
	else return a;
}
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
uint16_t barrel_ecd_watcher;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;

  uint8_t rx_data[26];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan == &hcan1)
  {
    /* 达妙 DM4310（大 yaw）反馈帧，靠 Master ID 匹配 */
    if (dm_motor_decode(&dm_4310, rx_header.StdId, rx_data))
    {
      return;
    }

    /* 大疆 GM6020（小 yaw）反馈帧 0x205~0x20B，只做记录，不 return */
    gimbal_6020_decode(&g6020, rx_header.StdId, rx_data);

    switch (rx_header.StdId)
    {
    case CAN_3508_M1_ID:
    {
      get_motor_measure(&motor_chassis[0], rx_data); //读取数据
      detect_hook(CHASSIS_MOTOR1_TOE); //错误检测
      break;
    }
    case CAN_3508_M2_ID:
    {
      get_motor_measure(&motor_chassis[1], rx_data);
      detect_hook(CHASSIS_MOTOR2_TOE);
      break;
    }
    case CAN_3508_M3_ID:
    {
      get_motor_measure(&motor_chassis[2], rx_data);
      detect_hook(CHASSIS_MOTOR3_TOE);
      break;
    }
    case CAN_3508_M4_ID:
    {
      get_motor_measure(&motor_chassis[3], rx_data);
      detect_hook(CHASSIS_MOTOR4_TOE);
      break;
    }
    case CAN_YAW_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[0], rx_data);
      // Calc_motor_Angle(&motor_gimbal[0]);
      detect_hook(YAW_GIMBAL_MOTOR_TOE);
      break;
    }
    case CAN_PIT_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[1], rx_data);
      // Calc_motor_Angle(&motor_gimbal[1]);
      detect_hook(PITCH_GIMBAL_MOTOR_TOE);
      break;
    }
    case CAN_TRIGGER_MOTOR_ID:
    {
      get_motor_measure(&motor_trigger, rx_data);
      Calc_motor_Angle(&motor_trigger);
      detect_hook(TRIGGER_MOTOR_TOE);
      break;
    }
    //  case CAN_LEFT_FRICTION_ID:
	//  	{
	//  		get_motor_measure(&shoot_control.left_friction.motor_measure, rx_data); //读取数据
	//  		detect_hook(LEFT_FRICTION_MOTOR_TOE);          							 //错误检测
	//  		break;
	//  	}
    //  case CAN_RIGHT_FRICTION_ID:
	//  	{
	//  		get_motor_measure(&shoot_control.right_friction.motor_measure, rx_data); //读取数据
	//  		detect_hook(RIGHT_FRICTION_MOTOR_TOE);         							 //错误检测
	//  		break;
	//  	}
    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hcan2)
  {
    switch (rx_header.StdId)
    {
		 case CAN_LEFT_FRICTION_ID:
		 {
		 	get_motor_measure(&shoot_control.left_friction.motor_measure, rx_data); //读取数据
		 	detect_hook(LEFT_FRICTION_MOTOR_TOE);          							 //错误检测
		 	break;
		 }
		 case CAN_RIGHT_FRICTION_ID:
		 {
		 	get_motor_measure(&shoot_control.right_friction.motor_measure, rx_data); //读取数据
		 	detect_hook(RIGHT_FRICTION_MOTOR_TOE);         							 //错误检测
		 	break;
		 }
		 case CAN_CHASSIS:
		 {
		 	if (rx_data[0] == 0xFF && rx_data[7] == 0xFE)
		 	{
		 	auto_chassis.vx_set = (rx_data[1]*0.1f-4.0f);//-4是为了保留移动速度的一位小数精度，协议上进行了数值偏移和扩大           
		 	auto_chassis.vy_set = (rx_data[2]*0.1f-4.0f);
		 	auto_chassis.wz_set = (rx_data[3]*0.1f-4.0f);
		 	auto_chassis.wz_flag = rx_data[4];//置一时wz由雷达控制
				
		 	auto_chassis.ramp_flag = rx_data[5];
		 	}
		 	break;
		 }
        case CAN_NAVIGATION:
        {
            	tx_buffer[0]=0xff;
				tx_buffer[1]=rx_data[0];
				tx_buffer[2]=rx_data[1];
				tx_buffer[3]=rx_data[2];
				tx_buffer[4]=rx_data[3];
				tx_buffer[5]=rx_data[4];
				tx_buffer[6]=rx_data[5];
				tx_buffer[7]=rx_data[6];
				tx_buffer[8]=rx_data[7];
            
        break;
        }
        case CAN_MESSAGE: 
        {
            	tx_buffer[9]=rx_data[0];
				tx_buffer[10]=rx_data[1];
				tx_buffer[11]=rx_data[2];
				tx_buffer[12]=rx_data[3];
				tx_buffer[13]=rx_data[4];
				tx_buffer[14]=rx_data[5];
				tx_buffer[15]=rx_data[6];
				tx_buffer[16]=rx_data[7];

        break;
        }
        case CAN_MESSAGE2:
        {
            
            tx_buffer[17]=rx_data[0];
			tx_buffer[18]=rx_data[1];
			tx_buffer[19]=rx_data[2];
			tx_buffer[20]=rx_data[3];
			tx_buffer[21]=rx_data[4];
			tx_buffer[22]=rx_data[5];
			tx_buffer[23]=rx_data[6];
			tx_buffer[24]=rx_data[7];
			tx_buffer[25]=0xfe;
            user_data_nv_pack0_handle_1();
        break;
        }
		default:
		{
			break;
		}
    }
  }
}


/**
 * @brief CAN转TTL
 * @param radar
 */


void CAN_cmd_NUC(uint8_t buf[8])
{   uint32_t send_mail_box;
    radar_tx_message.StdId = 0x001;//控制ID
    radar_tx_message.IDE = CAN_ID_STD;
    radar_tx_message.RTR = CAN_RTR_DATA;
    radar_tx_message.DLC = 0x08;
	Tx_Message[0] = buf[0];
	Tx_Message[1] = buf[1];
	Tx_Message[2] = buf[2];
	Tx_Message[3] = buf[3];
	Tx_Message[4] = buf[4];
	Tx_Message[5] = buf[5];
	Tx_Message[6] = buf[6];
	Tx_Message[7] = buf[7];
	// uint8_t TransmitMailbox = CAN_Transmit(CAN1, &Tx_Message);//发送数据
	HAL_CAN_AddTxMessage(&hcan2, &radar_tx_message, Tx_Message, &send_mail_box);
    // uint32_t timeout = 1000000; // 设置超时计数器 
    // while (CAN_TransmitStatus(CAN2, TransmitMailbox) != CAN_TxStatus_Ok)//超时退出，防止卡死
	// {
	// 	timeout--;
	// 	if (timeout == 0)
	// 	{
	// 		break;
	// 	}
	// }
}




void CAN_cmd_navigation(void)
{
#if GAME_MODE
  game_message.game_progress=game_state.game_progress;
  game_message.stage_remain_time=game_state.stage_remain_time;
  game_message.remain_HP=robot_state.remain_HP;
#endif
	
  float yaw=10000000*(gimbal_control.gimbal_yaw_motor.relative_angle);//给雷达发yaw轴数据，乘以100000000是为了保留精度
	
  uint32_t send_mail_box;
  radar_tx_message.StdId = CAN_NAVIGATION;
  radar_tx_message.IDE = CAN_ID_STD;
  radar_tx_message.RTR = CAN_RTR_DATA;
  radar_tx_message.DLC = 0x08;
  radar_can_send_data[0] = ((int32_t)(yaw) >> 24)&0xff;
  radar_can_send_data[1] = ((int32_t)(yaw) >> 16)&0xff;
  radar_can_send_data[2] = ((int32_t)(yaw) >>  8)&0xff;
  radar_can_send_data[3] = ((int32_t)(yaw) >>  0)&0xff;
  radar_can_send_data[4] = game_message.game_progress;
  radar_can_send_data[5] = (game_message.stage_remain_time >>  8);
  radar_can_send_data[6] = game_message.stage_remain_time;
  radar_can_send_data[7] = (game_message.remain_HP >> 8);
  HAL_CAN_AddTxMessage(&hcan2, &radar_tx_message, radar_can_send_data, &send_mail_box);
}

/**
 * @brief CAN转TTL（比赛信息）
 * @param radar
 */
/**
 * @brief CAN命令决策函数
 * 该函数负责根据游戏模式和当前状态生成CAN消息并发送
 */
void CAN_cmd_decesion(void)
{	
#if GAME_MODE  // 如果定义了游戏模式宏
  // 更新游戏消息中的子弹剩余数量
  game_message.bullet_remaining_num=bullet_remaining_t.bullet_remaining_num;
  // 更新游戏消息中的机器人ID
  game_message.robot_id=robot_state.robot_id;
  // 更新游戏消息中的蓝色基地血量
  game_message.blue_base_HP=game_robot_HP_t.blue_base_HP;
  // 更新游戏消息中的蓝色前哨站血量
  game_message.blue_outpost_HP=game_robot_HP_t.blue_outpost_HP;
  // 检查RFID基地状态
  bool_t a=rfid_status_t.rfid_status & (1<<19);
  // 将RFID基地状态更新到游戏消息中
  game_message.RFID_Base=a;
#endif
	
  uint32_t send_mail_box;  // 定义发送邮箱变量
  // 配置CAN消息的标准标识符
  radar_tx_message.StdId = CAN_MESSAGE;
  // 配置CAN消息为标准帧
  radar_tx_message.IDE = CAN_ID_STD;
  // 配置CAN消息为数据帧
  radar_tx_message.RTR = CAN_RTR_DATA;
  // 配置CAN数据长度为8字节
  radar_tx_message.DLC = 0x08;

  // 设置CAN发送数据数组的各个字节
  radar_can_send_data[0] = game_message.remain_HP;  // 剩余血量
  radar_can_send_data[1] = (game_message.bullet_remaining_num >> 8);  // 子弹数量高8位
  radar_can_send_data[2] = game_message.bullet_remaining_num;  // 子弹数量低8位
  radar_can_send_data[3] = game_message.RFID_Base;  // RFID基地状态
  radar_can_send_data[4] = (game_message.blue_base_HP>>8);  // 蓝色基地血量高8位
  radar_can_send_data[5] = game_message.blue_base_HP;  // 蓝色基地血量低8位
  radar_can_send_data[6] = (game_message.blue_outpost_HP>>8);  // 蓝色前哨站血量高8位
  radar_can_send_data[7] = game_message.blue_outpost_HP;
  HAL_CAN_AddTxMessage(&hcan2, &radar_tx_message, radar_can_send_data, &send_mail_box);
}

/**
 * @brief CAN转TTL（比赛信息）
 * @param radar
 */
void CAN_cmd_decesion2(void)
{
  float yaw2=10000000*(gimbal_control.gimbal_yaw_motor.absolute_angle);//给雷达发yaw轴数据，乘以100000000是为了保留精度

#if GAME_MODE
  game_message.red_base_HP=game_robot_HP_t.red_base_HP;
  game_message.red_outpost_HP=game_robot_HP_t.red_outpost_HP;
#endif
	
  uint32_t send_mail_box;
  radar_tx_message.StdId = CAN_MESSAGE2;
  radar_tx_message.IDE = CAN_ID_STD;
  radar_tx_message.RTR = CAN_RTR_DATA;
  radar_tx_message.DLC = 0x08;
  radar_can_send_data[0] = (game_message.red_base_HP>>8);
  radar_can_send_data[1] = game_message.red_base_HP;
  radar_can_send_data[2] = (game_message.red_outpost_HP>>8);
  radar_can_send_data[3] = game_message.red_outpost_HP;
  radar_can_send_data[4] = ((int32_t)(yaw2) >> 24)&0xff;
  radar_can_send_data[5] = ((int32_t)(yaw2) >> 16)&0xff;
  radar_can_send_data[6] = ((int32_t)(yaw2) >>  8)&0xff;
  radar_can_send_data[7] = ((int32_t)(yaw2) >>  0)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &radar_tx_message, radar_can_send_data, &send_mail_box);
}

/**
 * @brief 摩擦轮控制量发送函数 can2
 *
 * @param left_friction
 * @param rigit_friction
 * @param rev
 */
void CAN_cmd_fricion(int16_t left_friction, int16_t rigit_friction)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_FRICTION_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (left_friction >> 8);
  gimbal_can_send_data[1] = left_friction;
  gimbal_can_send_data[2] = (rigit_friction >> 8);
  gimbal_can_send_data[3] = rigit_friction;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief 拨弹电机控制量发送函数 can1
 *
 * @param shoot
 */
void CAN_cmd_shoot(int16_t shoot)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = 0x1FF;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208) can1
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal_yaw(int16_t yaw)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_YAW_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208) can1
 * @param[in]      pitch: (0x306) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal_pitch(int16_t pitch)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_PIT_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = pitch;
  gimbal_can_send_data[3] = (pitch >> 8);
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204) can1
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[0];
}

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[1];
}

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_radar_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[2];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_trigger;
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
