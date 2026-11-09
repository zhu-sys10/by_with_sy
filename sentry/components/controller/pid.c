/**
 * @file pid.c/h
 * @author 何泰宇、何清华
 * @brief pid实现函数，包括初始化，PID计算函数，PIID计算函数(引入变号情况Ki解决积分控制导致的震荡问题)
 * @version 2.0
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

inline float _abs(float a){
	if(a < 0) return -a;
	return a;
}

char HaveSameSign(float a, float b);

#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
	{                          \
		if (input > max)       \
		{                      \
			input = max;       \
		}                      \
		else if (input < -max) \
		{                      \
			input = -max;      \
		}                      \
	}

inline char HaveSameSign(float a, float b)
{
	if((a >= 0 && b >= 0) || (a <= 0 && b <= 0))
		return 1;
	else
		return 0;
}

/**
 * @brief          pid结构数据初始化
 * @param[out]     pid: PID结构数据指针
 * @param[in]      kd
 * @param[in]      ki
 * @param[in]      kd
 * @param[in]      max_out: pid最大输出
 * @param[in]      max_iout: pid最大积分输出
 */
extern float limit_k;

void PID_init(pid_type_def *pid, float Kp, float Ki, float Kd, float max_out, float max_iout)
{
	if (pid == NULL)
	{
		return;
	}
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief          pid计算
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @return         pid输出
 */
float PID_calc(pid_type_def *pid, float ref, float set)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid -> Ki * pid->error[0];
	/*if( HaveSameSign(pid->Iout, pid->error[0]) )
		pid->Iout += pid -> Ki * pid->error[0];
	else
		pid->Iout += pid -> Ki * pid->error[0] * 1.3f;*/
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout); //处理了积分饱和
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
	return pid->out;
}

/**
 * @brief          pitch轴  P={(Kp*err)^2*sgn(err),Kp2*err,Kp3*err} pid计算，带积分回复放大，P为x^2，给Δerr
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * @param[in]      err_delta: 误差值变化量
 * @return         pid输出
 */
float pitch_Px2_x_ID_calc_give_de(gimbal_PID_t *pid, float ref, float set, float err_delta)
{
	#define pitch_Kp2 16.0f
	#define pitch_Kp3 21.0f
	#define PITCH_ERR1 0.09f//第一分界点
	#define PITCH_ERR2 0.13f//第二分界点
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->set = set;
	pid->err = set - ref;
	if(_abs(pid->err) <= PITCH_ERR1)
	{//二次部
		pid->Pout = pid->kp * pid->err;
		pid->Pout *= pid->Pout;
		if(pid->err < 0) pid->Pout = - pid->Pout;
	}
	else if(_abs(pid->err) <= PITCH_ERR2)
	{//第一一次
		pid->Pout = pitch_Kp2 * pid->err;
	}
	else
	{//第二一次
		pid->Pout = pitch_Kp3 * pid->err;
	}
	//放大积分回复
	if( HaveSameSign(pid->Iout, pid->err) )
		pid->Iout += pid -> ki * pid->err;
	else
		pid->Iout += pid -> ki * pid->err * 1.27f;
	pid->Dout = pid->kd * err_delta;
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
	return pid->out;
}

/**
 * @brief          pid计算、但不覆写目标值
 * @param[out]     pid: PID结构数据指针
 * @param[in]      ref: 反馈数据
 * @return         pid输出
 */
float HTY_PID_calc(pid_type_def *pid, float ref)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->fdb = ref;
	pid->error[0] = pid->set - ref;
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
	return pid->out;
}


/**
 * @brief          pid 输出清除
 * @param[out]     pid: PID结构数据指针
 * @retval         none
 */
void PID_clear(pid_type_def *pid)
{
	if (pid == NULL)
	{
		return;
	}

	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
	pid->fdb = pid->set = 0.0f;
}
