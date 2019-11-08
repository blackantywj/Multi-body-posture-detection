#ifndef __ANO_PID_H
#define __ANO_PID_H

#include "stm32f10x.h"
#include "mymath.h"
#include "ANO_Config.h"

#if PID_Module
/*=====================================================================================================================
						   PID控制器设定参数
=====================================================================================================================*/
#define INNER_INTEGRAL    1.0 	//内环积分系数
#define INTEGRAL_LIMIT_EN 1		//开启积分限幅
/*=====================================================================================================================
						
=====================================================================================================================*/
typedef struct
{
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd;		 	 //微分系数
	float k_pre_d;       //previous_d 微分先行
	float k_ff;		     //前馈 
} PID_arg_t;

typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	float err_d;
	float err_i;
	float ff;
	float pre_d;

} PID_val_t;


void PID_calculate( float T,            //周期
					float in_ff,		//前馈
					float expect,		//期望值（设定值）
					float feedback,		//反馈值
					PID_arg_t *pid_arg, //PID参数结构体
					PID_val_t *pid_val,	//PID数据结构体
					float inte_lim,		//integration limit，积分限幅,0不限幅
					float *out  );		//输出


#endif //PID_Module
                    
#endif

