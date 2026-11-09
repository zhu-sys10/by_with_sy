/**
 * @file USART_receive.c
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 2.0
 * @date 2024-10-28
 *
 * @copyright Copyright (c) 2024
 *
 */
//度宝武.skill

#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "referee.h"
#include "string.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "INS_task.h"
#include "crc8_crc16.h"
#include "can_receive.h"

uint8_t debug_tx_buf[sizeof(TxData)];
TxData radar_tx_data1;
float gimbal_yaw_debug=0.0f;


uint8_t rxdebug_1[60]={0};
uint16_t len_th=0;
extern UART_HandleTypeDef huart1;
extern unsigned short NUC_GG_detect;
//radar
extern game_message_t game_message;
extern gimbal_control_t gimbal_control;
uint8_t radar_usart_send_data[24];


//新版雷达数据处理函数
// static TxData radar_tx_data;
// TxData radar_tx_data1;
// RxData new_auto_chassis_data;//接收
// void radar_receive_data_handle(uint8_t *buf);//接收
// void new_user_data_nv_pack_handle(void);//发送
void radar_receive_data_handle(uint8_t *buf);//接收
void new_user_data_nv_pack_handle(void);//发送
RxData new_auto_chassis_data;//接收



auto_shoot_t auto_shoot = {0.0f, 0.0f, 0}; //自瞄数据（废弃）
user_send_data_t user_send_data;
uint8_t usart_sizeof_size=0;

auto_target_t auto_target;

//中转代码的接收数据
uint8_t tx_buffer[26];
uint8_t tx_buf[sizeof(TxData)];

void user_data_radar_solve(volatile const uint8_t *buf, auto_chassis_t *chassis_ptr);

//接收原始数据，为10个字节，给了18鲎纸诔ざ龋乐笵MA传输越界
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

//static uint8_t radar_usart_send_data[8];
/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_target_t *auto_target);

/**
 * @brief 上位机串口初始化
 * @param[out] void
 */
void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{

    if (huart1.Instance->SR & UART_FLAG_RXNE) //接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区1
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            len_th=this_time_rx_len;
            if (this_time_rx_len == sizeof(new_auto_chassis_data))
            {
			    radar_receive_data_handle(usart1_rx_buf[0]);
            }
						memcpy(rxdebug_1,usart1_rx_buf[0],this_time_rx_len);
            // else if (this_time_rx_len == USER_FRAME_LENGTH)
            // {
              
            // }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(huart1.hdmarx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
            //重新设定数据长度
            huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
            //设定缓冲区0
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(huart1.hdmarx);
            if (this_time_rx_len == sizeof(new_auto_chassis_data))
            {
			    radar_receive_data_handle(usart1_rx_buf[1]);
            }
            // else if (this_time_rx_len == USER_FRAME_LENGTH)
            // {
            
            // }
        }
    }
}




void user_data_radar_solve(volatile const uint8_t *buf, auto_chassis_t *chassis_ptr)
{
     uint8_t rx_data[8];
     memcpy(rx_data, (void*)buf, 8);
    if (rx_data[0] == 0xFF && rx_data[7] == 0xFE)
    {
        chassis_ptr->vx_set = (rx_data[1] * 0.1f - 4.0f); /* -4 协议偏移 */
        chassis_ptr->vy_set = (rx_data[2] * 0.1f - 4.0f);
        chassis_ptr->wz_set = (rx_data[3] * 0.1f - 4.0f);
        chassis_ptr->wz_flag = rx_data[4]; /* 标志位 */
        chassis_ptr->ramp_flag = rx_data[5];
        chassis_ptr->new_chassis_data_flag = 1; // 添加这一行！
    }
}



/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_target_t *auto_target)
{
	static uint8_t buf_debug[16];
	memcpy(buf_debug, (void*)buf, sizeof(buf_debug)); 
    if (buf_debug[0] == 0xFF && buf_debug[15] == 0xFE)
    {   
		auto_target->fire = buf_debug[1];
		
		memcpy(&auto_target->yaw, buf_debug+2, sizeof(float));//从第三个数据开始复制
		memcpy(&auto_target->pitch, buf_debug+6,sizeof(float));
		memcpy(&auto_target->distance, buf_debug+10,sizeof(float));
		
		auto_target->enemy_id = buf_debug[1];//暂时没用
		auto_target->transmit_good = 1;
		detect_hook(USER_USART_DATA_TOE);
		
		NUC_GG_detect = 0;
    }
}
/**
 * @brief 给自瞄上位机发送数据
 * @param[out] void
 * @param user_send_data 发送的自瞄数据
 */




 /***********************新版雷达发送协议******************************** */
void new_user_data_nv_pack_handle(void)
{   

    //static uint8_t tx_buf[sizeof(TxData)];
    static TxData radar_tx_data;
#if GAME_MODE
    game_message.game_progress   = game_state.game_progress;
    game_message.stage_remain_time = game_state.stage_remain_time;
    game_message.remain_HP       = robot_state.remain_HP;
#endif

    radar_tx_data.header                = 0xFF;
    radar_tx_data.gimbal_yaw            = gimbal_control.gimbal_yaw_motor.relative_angle;//3.0;//gimbal_control.gimbal_yaw_motor.relative_angle;
    radar_tx_data.game_progress         = 1;//game_message.game_progress;
    radar_tx_data.stage_remain_time     = 100;//game_message.stage_remain_time;
    radar_tx_data.current_hp            = 80;//game_message.remain_HP;
    radar_tx_data.bullet_allowance_17mm = game_message.bullet_remaining_num;

    radar_tx_data.rfid                  = 1.0;//game_message.RFID_Base;
    radar_tx_data.blue_base_hp          = 4000;//game_message.blue_base_HP;
    radar_tx_data.blue_outpost_hp       = 1000;//game_message.blue_outpost_HP;
    radar_tx_data.red_base_hp           = 400;//game_message.red_base_HP;
    radar_tx_data.red_outpost_hp        = 500;//game_message.red_outpost_HP;
    radar_tx_data.ender                = 0xFE;
    // 串口DMA发送结构体
   memcpy(debug_tx_buf, &radar_tx_data, sizeof(TxData));//debug
    gimbal_yaw_debug=radar_tx_data.gimbal_yaw;
    memcpy(&radar_tx_data1, &radar_tx_data, sizeof(TxData));
    usart1_tx_dma_enable((uint8_t *)&radar_tx_data, sizeof(TxData));
}

//雷达接收函数
void radar_receive_data_handle(uint8_t *buf)
{
//后续添加帧头帧尾校验，
    memcpy(&new_auto_chassis_data, buf, sizeof(RxData));
//else memny_set(0)//否则清除内存
}



/************ */              //旧版雷达发送函数/*********************************** */
void user_data_nv_pack0_handle(void){

    //static uint8_t tx_buf[26];
    
#if GAME_MODE
  game_message.game_progress=game_state.game_progress;
  game_message.stage_remain_time=game_state.stage_remain_time;
  game_message.remain_HP=robot_state.remain_HP;
#endif
    float yaw=10000000*gimbal_control.gimbal_yaw_motor.relative_angle;
    radar_usart_send_data[0] = 0xff;
    radar_usart_send_data[1] = ((int32_t)(yaw) >> 24)&0xff;
    radar_usart_send_data[2] = ((int32_t)(yaw) >> 16)&0xff;
    radar_usart_send_data[3] = ((int32_t)(yaw) >>  8)&0xff;
    radar_usart_send_data[4] = ((int32_t)(yaw) >>  0)&0xff;
    radar_usart_send_data[5] = 1;//(game_message.game_progress >> 8);
    radar_usart_send_data[6] = (game_message.stage_remain_time >>  8);
    radar_usart_send_data[7] = game_message.stage_remain_time;
    radar_usart_send_data[8] = (game_message.remain_HP >> 8);
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
	radar_usart_send_data[9] = game_message.remain_HP;  // 剩余血量
    radar_usart_send_data[10] = (game_message.bullet_remaining_num >> 8);  // 子弹数量高8位
    radar_usart_send_data[11] = game_message.bullet_remaining_num;  // 子弹数量低8位
    radar_usart_send_data[12] = game_message.RFID_Base;  // RFID基地状态
    radar_usart_send_data[13] = (game_message.blue_base_HP>>8);  // 蓝色基地血量高8位
    radar_usart_send_data[14] = game_message.blue_base_HP;  // 蓝色基地血量低8位
    radar_usart_send_data[15] = (game_message.blue_outpost_HP>>8);  // 蓝色前哨站血量高8位
    radar_usart_send_data[16] = game_message.blue_outpost_HP;  // 蓝色前哨站血量低8位
#if GAME_MODE
  game_message.red_base_HP=game_robot_HP_t.red_base_HP;
  game_message.red_outpost_HP=game_robot_HP_t.red_outpost_HP;
#endif
    float yaw2=10000000*(gimbal_control.gimbal_yaw_motor.absolute_angle);//给雷达发yaw轴数据，乘以100000000是为了保留精度
    radar_usart_send_data[17] = (game_message.red_base_HP>>8);
    radar_usart_send_data[18] = game_message.red_base_HP;
    radar_usart_send_data[19] = (game_message.red_outpost_HP>>8);
    radar_usart_send_data[20] = game_message.red_outpost_HP;
    radar_usart_send_data[21] = ((int32_t)(yaw2) >> 24)&0xff;
    radar_usart_send_data[22] = ((int32_t)(yaw2) >> 16)&0xff;
    radar_usart_send_data[23] = ((int32_t)(yaw2) >>  8)&0xff;
    radar_usart_send_data[24] = ((int32_t)(yaw2) >>  0)&0xff;
    radar_usart_send_data[25] = 0xfe;    
    memcpy(tx_buf, radar_usart_send_data, 26);
	memcpy(debug_tx_buf,radar_usart_send_data,26);
	//append_CRC16_check_sum(tx_buf,sizeof(user_send_data_t));//不用CRC，防止冒出0xFF或0xFE影响视觉那边判断
    usart1_tx_dma_enable(tx_buf, 26);
}

/**
 * @brief 给自瞄上位机发送数据
 * @param[out] void
 * @param user_send_data 发送的自瞄数据
 */

void user_data_nv_pack0_handle_1(void){
    static uint8_t tx_buf_1[26];
    memcpy(tx_buf_1,tx_buffer , 26);
	memcpy(debug_tx_buf,tx_buffer,26);
	//append_CRC16_check_sum(tx_buf,sizeof(user_send_data_t));//不用CRC，防止冒出0xFF或0xFE影响视觉那边判断
    usart1_tx_dma_enable(tx_buf_1, 26);
}


void user_data_pack_handle()
{
	// static uint8_t tx_buf[sizeof(user_send_data_t)+2];
	static uint8_t tx_buf[sizeof(user_send_data_t)];

    user_send_data.header=0xFF;
	//user_send_data.enemy_color = 1;//0是自己蓝，对面红
    if (get_robot_id() <= 7)
    {
        user_send_data.enemy_color = 1; //蓝
    }
    else
    {
        user_send_data.enemy_color = 0; //红
    }
	user_send_data.pitch=INS_data.angle_pitch;
	user_send_data.yaw=INS_data.angle_yaw;
	user_send_data.box1=0;
	user_send_data.box2=0;
	user_send_data.box3=0;
	user_send_data.end = 0xFE;
	memcpy(tx_buf, &user_send_data,sizeof(user_send_data_t));
	//append_CRC16_check_sum(tx_buf,sizeof(user_send_data_t));//不用CRC，防止冒出0xFF或0xFE影响视觉那边判断
	//usart1_tx_dma_enable(tx_buf,sizeof(user_send_data_t)+2);
    // usart1_tx_dma_enable(tx_buf,sizeof(user_send_data_t)+2);
    usart1_tx_dma_enable(tx_buf,sizeof(user_send_data_t));
    usart_sizeof_size=sizeof(user_send_data_t);

//    usart_sizeof_size=sizeof(user_send_data_t)+2;
 
}

