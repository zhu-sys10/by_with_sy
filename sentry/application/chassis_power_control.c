/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       对于底盘功率进行拟合，使用拟合函数预测底盘功率并进行功率限制
  * @history
  *  Version    Date            Author          Modification
  *  Vn       26-3-2024        bobolin           
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"

/**
 * @brief          检测转速异常数据
 * @param[in]      a+b+c
 * @retval         none
 */
float LIMIT_MAX_MIN(float x, float max, float min);

/**
 * @brief          限制功率，主要限制电机电流
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
 
/**
 * @brief 底盘功率控制函数
 * @param chassis_power_control 底盘运动控制结构体指针
 * @note 该函数用于计算底盘功率预测并进行超功率衰减处理
 */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    // 发送期望转速之后产生的功率预测
    // abc为二次函数系数
    float a = 0, b = 0, c = 0;
    for (int i = 0; i < 4; i++)
    {
        a += MOTOR_3508_R * chassis_power_control->k[i] * chassis_power_control->k[i] * chassis_power_control->motor_chassis[i].speed_rpm_set * chassis_power_control->motor_chassis[i].speed_rpm_set;
        b += chassis_power_control->motor_chassis[i].speed_rpm_set * (2 * MOTOR_3508_R * chassis_power_control->k[i] * chassis_power_control->m[i] +
                                                                      MOTOR_3508_K * chassis_power_control->k[i] * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm);
        c += (MOTOR_3508_R * chassis_power_control->m[i] * chassis_power_control->m[i] +
              MOTOR_3508_K * chassis_power_control->m[i] * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm);
    }
    c += MOTOR_3508_P;//+静息损耗
    // 这里相当于k_c=1时，预测功率 ,也就是当转速不衰减时，预测功率
    chassis_power_control->predict_send_power = LIMIT_MAX_MIN(a + b + c, 1000, -1000); // 过滤转速预测异常数据
    
    
    // 超功率衰减
    if (chassis_power_control->predict_send_power > chassis_power_control->chassis_power_set)
    {
        // 模型衰减计算
        if (b * b < 4 * (c - chassis_power_control->chassis_power_set) * a) // b方小于4ac：没有解
        {
            chassis_power_control->limit_k = LIMIT_MAX_MIN(-b / (2 * a), 1.0, 0.0);
        }
        else
        {
            float32_t sqrt_result;
            arm_sqrt_f32(b * b - 4 * (c - chassis_power_control->chassis_power_set) * a, &sqrt_result);
			if(a != 0)
            chassis_power_control->limit_k = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a, 1.0, 0.0);//取最大速度
			else
			chassis_power_control->limit_k = 0.0f;
        }
        // 衰减
        for (int i = 0; i < 4; i++)
        {
            chassis_power_control->motor_chassis[i].speed_rpm_set *= chassis_power_control->limit_k; // 衰减
        }
    }
}

/**
 * @brief          对于x进行限制
 * @param[in]      a+b+c
 * @retval         none
 */
float LIMIT_MAX_MIN(float x, float max, float min)
{
    if (x > max)
    {
        return max;
    }
    else if (x < min)
    {
        return min;
    }
    else
    {
        return x;
    }
}

/**
 * @brief          从pid控制算法中反解转速和电流映射关系常数K，M
 * @param[in]      “chassis_power_control”底盘功率控制指针
 */

void calfromPID(pid_type_def *power_limit ,chassis_move_t *chassis_power_control)
{
//    float k_s_min;
//    k_s_min = 1.0;
//    for (int i = 0; i < 4; i++)
//    {
//        pid_type_def *P = &(chassis_power_control->motor_speed_pid[i]);
//        short speed_now = chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
//        float PreError = P->set - speed_now;
//        float SumError_last = P->error[0];
//			//记录error[0];
//	//		float SumError_last;
//	//		if(P->Ki != 0)
//	//		{
//	//			SumError_last=P->Iout/P->Ki;
//	//		}
//	//		else
//	//		{
//	//			SumError_last = P->error[0];
//	//		}
//        float SumError = SumError_last;
//			//比其他兵种多一个这个if的判断
//			if(PreError > -(P->max_iout/P->Ki) && PreError < (P->max_iout/P->Ki))
//			{
//			SumError += (P->error[0] + P->error[1]) / 2; // 梯形积分提高精度
//			}
//        if (fabs(SumError) >= P->max_iout) // 不积分的情况
//        {
//            chassis_power_control->m[i] = (-P->Kp * speed_now + P->Ki * SumError_last) * RM3508_CURRENT_RATIO;
//            chassis_power_control->k[i] = P->Kp * RM3508_CURRENT_RATIO;
//        }
//        else
//        {
//            chassis_power_control->m[i] = (-P->Kp * speed_now + P->Ki * (SumError_last + 0.5f * (-speed_now + P->error[0]))) * RM3508_CURRENT_RATIO;
//            chassis_power_control->k[i] = (P->Kp + P->Ki / 2) * RM3508_CURRENT_RATIO;
//        }
//        double k_s = 1.0;
//        double K_W = chassis_power_control->k[i] * P->set;
//        double current = K_W * k_s + chassis_power_control->m[i];
//        if (fabs(current) > chassis_power_control -> motor_speed_pid[i].max_out * RM3508_CURRENT_RATIO)
//        {
//            double l = current > 0 ? 1 : -1;
//            k_s = LIMIT_MAX_MIN((l * chassis_power_control -> motor_speed_pid[i].max_out * RM3508_CURRENT_RATIO - chassis_power_control->m[i]) /K_W,
//                                1.0, 0.0);
//            if (k_s < k_s_min)
//            {
//                k_s_min = k_s;
//            }
//        }
//    }
//    for (int i = 0; i < 4; i++)
//    {
//        chassis_power_control->motor_speed_pid[i].set *= k_s_min; // 最大值越界衰减
//    }
	
	    float k_s_min;
    k_s_min = 1.0;
    for (int i = 0; i < 4; i++)
    {
        pid_type_def *P = &(chassis_power_control->motor_speed_pid[i]);
        short speed_now = chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
        float PreError = P->set - speed_now;
        float SumError_last = P->error[0];
        float SumError = SumError_last;

        SumError += (P->error[0] + P->error[1]) / 2; // 梯形积分提高精度

        if (fabs(SumError) >= P->max_iout) // 不积分的情况
        {
            chassis_power_control->m[i] = (-P->Kp * speed_now + P->Ki * SumError_last) * RM3508_CURRENT_RATIO;
            chassis_power_control->k[i] = P->Kp * RM3508_CURRENT_RATIO;
        }
        else
        {
            chassis_power_control->m[i] = (-P->Kp * speed_now + P->Ki * (SumError_last + 0.5f * (-speed_now + P->error[0]))) * RM3508_CURRENT_RATIO;
            chassis_power_control->k[i] = (P->Kp + P->Ki / 2) * RM3508_CURRENT_RATIO;
        }
        double k_s = 1.0;
        double K_W = chassis_power_control->k[i] * P->set;
        double current = K_W * k_s + chassis_power_control->m[i];
        if (fabs(current) > chassis_power_control -> motor_speed_pid[i].max_out * RM3508_CURRENT_RATIO)
        {
            double l = current > 0 ? 1 : -1;
            k_s = LIMIT_MAX_MIN((l * chassis_power_control -> motor_speed_pid[i].max_out * RM3508_CURRENT_RATIO - chassis_power_control->m[i]) /K_W,
                                1.0, 0.0);
            if (k_s < k_s_min)
            {
                k_s_min = k_s;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        chassis_power_control->motor_speed_pid[i].set *= k_s_min; // 最大值越界衰减
    }
}
