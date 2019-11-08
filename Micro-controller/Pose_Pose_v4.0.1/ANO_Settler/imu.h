#ifndef _IMU_H_
#define	_IMU_H_

#include "stm32f10x.h"

#include "include.h"
#include "mymath.h"
#include "POSE_Drv_MPU9250.h"
#include "filter.h"
#include <math.h>		/*¿âº¯Êý*/

#if POSE_IMU

#ifndef PI
#define PI					3.14159265358979f
#endif

//60×Ö½Ú
typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;

extern xyz_f_t reference_v,acc_3d_hg;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *Q);
void IMU_getInfo(float dt);
void Kalman_IMMU_Init(void);
void Kalman_IMMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q, float dt);
extern float Roll,Pitch,Yaw;
extern float Roll_ANO, Pitch_ANO, Yaw_ANO;
extern float Roll_NEW_XILUNA, Pitch_NEW_XILUNA, Yaw_NEW_XILUNA;
extern float ewx, ewy, ewz;

#endif


#endif

