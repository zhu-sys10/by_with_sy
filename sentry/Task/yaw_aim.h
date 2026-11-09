/**
  * @file   yaw_aim.h
  * @brief  世界角瞄准：IMU 给绝对世界角，小 yaw 锁角，大 yaw 回中
  *
  * 世界角以 IMU 为准（上电位置为 0），不用编码器推算。
  * 小 yaw 目标 = 当前角 + 世界误差：
  *   - 给新目标时大 yaw 还没动，小 yaw 独吃误差 -> 迅速到位
  *   - 大 yaw 追过来，世界误差变小，小 yaw 目标随之缩小 -> 回中
  *   - 大 yaw 被摇杆转走，世界误差自动反向 -> 世界角不变
  */

#ifndef __YAW_AIM_H__
#define __YAW_AIM_H__

/* 小 yaw 软限幅 rad，30°。物理 ±1.05，留机械余量 */
#define YAW_AIM_SMALL_LIMIT   (0.5235988f)

/* 大 yaw 自动回中：最大速率 rad/s，比例增益 1/s */
#define YAW_AIM_CENTER_SLEW   (6.0f)
#define YAW_AIM_CENTER_GAIN   (3.0f)

/* 调试口空闲值 */
#define YAW_AIM_CMD_IDLE      (-999.0f)

/* 调试口：Ozone 里写目标角（度），生效一次后自动复位 */
extern volatile float yaw_aim_cmd_deg;

/* 当前目标世界角 rad，只读 */
extern volatile float yaw_aim_target_rad;

/* 进入瞄准：把当前世界角锁成目标（防止拨杆瞬间甩动）*/
void yaw_aim_enter(float world_yaw, float big_yaw);

void  yaw_aim_set_target(float world_yaw);
float yaw_aim_get_target(void);

/**
  * @brief  每拍调用
  * @param[in]  dt              周期 s
  * @param[in]  world_yaw       世界角 rad（IMU angle_yaw）
  * @param[in]  small_yaw       小 yaw 相对角 rad（6020 编码器）
  * @param[in]  big_yaw         大 yaw 轴角 rad
  * @param[in]  big_manual_rate 左摇杆给的大 yaw 角速度 rad/s，0 表示不接管
  * @param[out] big_cmd         大 yaw 目标 rad
  * @param[out] big_rate        大 yaw 参考角速度 rad/s（速度前馈）
  * @param[out] small_cmd       小 yaw 目标 rad（相对大 yaw）
  */
void yaw_aim_update(float dt, float world_yaw, float small_yaw, float big_yaw,
                    float big_manual_rate, float *big_cmd, float *big_rate,
                    float *small_cmd);

#endif
