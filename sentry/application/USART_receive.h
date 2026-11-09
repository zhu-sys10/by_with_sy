/**
 * @file USART_receive.h
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 32u
#define USER_FRAME_LENGTH 8u//9u//16u

#define USART_PI 3.1416f



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
}game_message_t;



//========================雷达相关==========================
/*************** 上位机发送给下位机 ***************/
typedef struct
{
    float linear_x; 
    float linear_y;
    //float angular_z;
    uint8_t status;
}RxData;

/*************** 下位机发送给上位机 ***************/
typedef __packed struct
{   uint8_t header;
    float gimbal_yaw;                 // 云台yaw角度
    uint8_t game_progress;           // 比赛阶段
    uint16_t stage_remain_time;      // 剩余时间
    uint16_t current_hp;             // 当前血量
    uint16_t bullet_allowance_17mm;  // 子弹数量
    uint8_t rfid;                    // RFID状态
    uint16_t blue_base_hp;           
    uint16_t blue_outpost_hp;
    uint16_t red_base_hp;
    uint16_t red_outpost_hp;
    uint8_t ender;
}TxData;




typedef struct
{
    float pitch_add;
    float yaw_add;
    bool_t new_usart_data_flag;
} auto_shoot_t;

typedef struct
{
	char fire;
	float yaw;//方位
	float pitch; //pitch角度
	float distance;
	
	int enemy_id;//暂时没用到
	char transmit_good;//判断是否受到数据
} auto_target_t;

typedef struct
{
	uint8_t header;
	uint8_t enemy_color;
	
	float pitch;
	float yaw;	
	 uint8_t box1;
	uint8_t box2;
	uint8_t box3;
	uint8_t end;
} user_send_data_t;//发送时莫名奇妙变成十六位数据

extern auto_shoot_t auto_shoot;
extern auto_target_t auto_target;
extern user_send_data_t user_send_data;
extern void user_usart_init(void);
extern void user_data_pack_handle(void);
//extern void auto_nv__handle(void);
extern void user_data_nv_pack0_handle_1(void);
// extern void user_data_nv_pack0_handle_1(void);
extern void user_data_nv_pack0_handle(void);
#endif
