/**
 * @file config.h
 * @author 何清华；周鸣阳; 李浩; 何泰宇
 * @brief 预编译设置
 * @version 2.0
 * @date 2024-10-28
 * 
 * @copyright Copyright (c) 2024 SPR
 * 
 */

#ifndef __CONFIG_H
#define __CONFIG_H

//线程开关
//#define CHASSIS_TASK	//底盘线程
//#define GIMBAL_TASK		//云台线程
//#define REFEREE_TASK	//裁判系统收发线程
// #define SHOOT_TASK		//发射线程
#define DETECT_TASK		//错误检测线程
#define INS_TASK		//姿态传感器线程
//#define RADAR_TASK//send data to radar

#define INS_PITCH_TURN 					0	//陀螺仪pitch角测量反向（C板倒着装时打开）
#define CHASSIS_ZERO_CURRENT 			0   //置为1后底盘电机0电流输出
#define GIMBAL_PITCH_ZERO_CURRENT 		0	//置为1后云台pitch电机0电流输出
#define GIMBAL_YAW_ZERO_CURRENT 		0	//置为1后云台YAW电流输出为0

#define GAME_MODE						1   //置1，拨杆左上角为比赛模式，置0，拨杆左上角底盘云台直接不动
#define USART6_RADAR_FLAG				0   //置1，调试模式，置0，正常模式
#endif


