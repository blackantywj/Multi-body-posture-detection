/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：imu.c
 * 描述    ：姿态解算
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "imu.h"
#include "include.h"
#include "POSE_Drv_MPU9250.h"
#include "mymath.h"
#include "filter.h"
#include <math.h>		/*库函数*/

#define Kp 0.3f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1			//(Hz)

#if POSE_IMU

xyz_f_t reference_v;
ref_t 	ref;

float Roll, Pitch, Yaw;    				//姿态角
float Roll_ANO, Pitch_ANO, Yaw_ANO;    				      //姿态角
float Roll_NEW_XILUNA, Pitch_NEW_XILUNA, Yaw_NEW_XILUNA;    //姿态角
/*----------------------------------------------------
Roll：  -180°~ 180°
Pitch:  -90° ~ 90°
Yaw:    -180°~ 180°	(0°指向北方)
----------------------------------------------------*/
float ref_q[4] = { 1,0,0,0 };
float norm_acc, norm_q;
float norm_acc_lpf;

float mag_norm, mag_norm_xyz;

xyz_f_t mag_sim_3d, acc_3d_hg, acc_ng, acc_ng_offset;

u8 acc_ng_cali;
u8 imu_ready = 0;
//得到Roll,Pitch,Yaw
void IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay, float az, float *rol, float *pit, float *yaw)
{
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;

	mag_norm_tmp = 20 * (6.28f *half_T);	   /*0.2520792*/

	/*磁力计的模*/
	mag_norm_xyz = my_sqrt(mpu9250.Mag_Val.x * mpu9250.Mag_Val.x + mpu9250.Mag_Val.y * mpu9250.Mag_Val.y + mpu9250.Mag_Val.z * mpu9250.Mag_Val.z);

	if (mag_norm_xyz != 0) /*归一化后积分？*/
	{
		mag_tmp.x += mag_norm_tmp *((float)mpu9250.Mag_Val.x / (mag_norm_xyz)-mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *((float)mpu9250.Mag_Val.y / (mag_norm_xyz)-mag_tmp.y);
		mag_tmp.z += mag_norm_tmp *((float)mpu9250.Mag_Val.z / (mag_norm_xyz)-mag_tmp.z);
	}

	/*
	void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out)

	罗盘数据是机体坐标下的，且磁场方向不是平行于地面，如果飞机倾斜，投影计算的角度会存在误差。
	此函数可在一定范围内做近似转换，让结果逼近实际角度，减小飞机倾斜的影响。
	注意：该函数内的计算并不是正确也不是准确的，正确的计算相对复杂，这里不给出，在未来的版本中会再更新。
	*/
	simple_3d_trans(&reference_v, &mag_tmp, &mag_sim_3d);  /*mag_sim_3d->算出较为正确的磁力方向*/
/*
磁力计的坐标是这样(从上往下，右手坐标系)
			   个
				|
				|y
		 -------+------>
				| x	(正方向)
				|
*/
/*较为正确的磁力向量的模*/
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);

	if (mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag = fast_atan2((mag_sim_3d.y / mag_norm), (mag_sim_3d.x / mag_norm)) *57.3f;   /*通过磁力计求出飞控的偏航角的基准(指向北)*/

	}
	//=============================================================================
	//四元数性质:ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3] = 1;
	// 计算等效(理论)重力向量(模是1->单位向量(只关心理论重力向量的方向，不关心大小))	 矩阵:C(b,R) = (C(R,b))^T
	reference_v.x = 2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]);
	reference_v.y = 2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]);
	reference_v.z = 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];


	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
	//=============================================================================

	if (acc_ng_cali)		 /*解锁后校准*/
	{
		if (acc_ng_cali == 2)
		{
			acc_ng_offset.x = 0;
			acc_ng_offset.y = 0;
			acc_ng_offset.z = 0;
		}

		acc_ng_offset.x += 10 * TO_M_S2 *(ax - 4096 * reference_v.x) *0.0125f;
		acc_ng_offset.y += 10 * TO_M_S2 *(ay - 4096 * reference_v.y) *0.0125f;
		acc_ng_offset.z += 10 * TO_M_S2 *(az - 4096 * reference_v.z) *0.0125f;

		acc_ng_cali++;
		if (acc_ng_cali >= 82) //start on 2	  (160ms)
		{
			acc_ng_cali = 0;
		}
	}

	acc_ng.x = 10 * TO_M_S2 *(ax - 4096 * reference_v.x) - acc_ng_offset.x; /*矫正加速度，并换算单位成mm/s^2*/
	acc_ng.y = 10 * TO_M_S2 *(ay - 4096 * reference_v.y) - acc_ng_offset.y; /*矫正加速度，并换算单位成mm/s^2*/
	acc_ng.z = 10 * TO_M_S2 *(az - 4096 * reference_v.z) - acc_ng_offset.z; /*矫正加速度，并换算单位成mm/s^2*/

	acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;	/*acc_3d_hg.z 约等于 acc_ng.z ,用来超声波融合*/


	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);

	/*通过加速度传感器获得的角度信息对陀螺仪的角度信息的积累误差进行校正*/
	if (ABS(ax) < 4400 && ABS(ay) < 4400 && ABS(az) < 4400)
	{
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 

		if (3800 < norm_acc && norm_acc < 4400)
		{
			/* 单位重力加速度向量叉乘得到角度误差(在角度θ小于45°时,sinθ = θ) */
			//ax,ay,az:	实际单位重力加速度向量(加速度计)
			//reference_v.x,reference_v.y,reference_v.z:理论单位重力加速度向量(四元数换算成《方向余弦矩阵》中的第三列的三个元素)
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
			//ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;  (Z轴(偏航角)会飘)

			  /* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *(ref.err_tmp.x - ref.err_lpf.x);
			ref.err_lpf.y += ref_err_lpf_hz *(ref.err_tmp.y - ref.err_lpf.y);
			//ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );

			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
		  //ref.err.z = ref.err_lpf.z ;
		}
	}
	else
	{
		ref.err.x = 0;
		ref.err.y = 0;
		//		ref.err.z = 0 ;
	}
	/* 角度误差积分 */
	ref.err_Int.x += ref.err.x *Ki * 2 * half_T;	  /*T = 2 *half_T;*/
	ref.err_Int.y += ref.err.y *Ki * 2 * half_T;
	ref.err_Int.z += ref.err.z *Ki * 2 * half_T;

	/* 角度积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.y = LIMIT(ref.err_Int.y, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
	ref.err_Int.z = LIMIT(ref.err_Int.z, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

	if (reference_v.z > 0.0f)
	{
		if (imu_ready/*fly_ready*/)
		{
			yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - Yaw);
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp *1.5f *To_180_degrees(yaw_mag - Yaw);
			//没有解锁，视作开机时刻，快速纠正
		}

	}
	else
	{
		yaw_correct = 0; //角度过大，停止修正，修正的目标值可能不正确
	}

	/*PI修正陀螺仪(mpu6050.Gyro_deg),单位：弧度*/
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + (Kp*(ref.err.x + ref.err_Int.x));     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + (Kp*(ref.err.y + ref.err_Int.y));		//IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;

	/* 用叉积误差来做PI修正陀螺零偏 */

	/*四元数的求解：*/
	// integrate quaternion rate and normalise	/*使用一阶龙格库塔（Runge-Kutta）更新四元数*/
	ref_q[0] = ref_q[0] + (-ref_q[1] * ref.g.x - ref_q[2] * ref.g.y - ref_q[3] * ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0] * ref.g.x + ref_q[2] * ref.g.z - ref_q[3] * ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0] * ref.g.y - ref_q[1] * ref.g.z + ref_q[3] * ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0] * ref.g.z + ref_q[1] * ref.g.y - ref_q[2] * ref.g.x)*half_T;

	/* 四元数规一化 (只要旋转信息，不平移)normalise quaternion */
	norm_q = my_sqrt(ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2] + ref_q[3] * ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;

	/*求Roll/Pitch/Yaw,Z-Y-X欧拉角顺序*/
	//*rol = fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *57.3f;	   /*由此式可知以飞控前侧为X轴*/
	//*pit = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2])) *57.3f;	/*由此式可知以飞控左侧为Y轴，没有负号?*/
	*pit = fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]), 1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *57.3f;
	*rol = asin(2 * (ref_q[0] * ref_q[2] - ref_q[1] * ref_q[3])) *57.3f;
	*yaw = fast_atan2(2 * (-ref_q[1] * ref_q[2] - ref_q[0] * ref_q[3]), 2 * (ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1]) - 1)*57.3f;

}


//---------------------------------------------------------------------------------------------------
// Definitions
#define sampleFreq	500.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMU_getInfo(float dt)
{
	static float Q[4];
	static float new_Q[4];
	static float angles[6];
	//更新四元数(6轴IMU-Inertial Measurement Units)
	//MahonyAHRSupdate(mpu9250.Gyro_deg.x * PI / 180, mpu9250.Gyro_deg.y * PI / 180, mpu9250.Gyro_deg.z * PI / 180,	 // PI/180：角度转弧度
	//	mpu9250.Acc.x / 4096, mpu9250.Acc.y / 4096, mpu9250.Acc.z / 4096, mpu9250.Mag_Val.x, mpu9250.Mag_Val.y, mpu9250.Mag_Val.z, new_Q);

	//MadgwickAHRSupdate(mpu9250.Gyro_deg.x * PI / 180, mpu9250.Gyro_deg.y * PI / 180, mpu9250.Gyro_deg.z * PI / 180,	 // PI/180：角度转弧度
	//	mpu9250.Acc.x / 4096, mpu9250.Acc.y / 4096, mpu9250.Acc.z / 4096, mpu9250.Mag_Val.x, mpu9250.Mag_Val.y, mpu9250.Mag_Val.z, Q/*new_Q*/);

	//MadgwickAHRSupdate(mpu9250.Gyro_deg.x * PI / 180, mpu9250.Gyro_deg.y * PI / 180, mpu9250.Gyro_deg.z * PI / 180,	 // PI/180：角度转弧度
	//	mpu9250.Acc.x / 4096, mpu9250.Acc.y / 4096, mpu9250.Acc.z / 4096, 0, 0, 0, Q/*new_Q*/);

	Kalman_IMMU(mpu9250.Gyro_deg.x * PI / 180, mpu9250.Gyro_deg.y * PI / 180, mpu9250.Gyro_deg.z * PI / 180,	 // PI/180：角度转弧度
		mpu9250.Acc.x / 4096, mpu9250.Acc.y / 4096, mpu9250.Acc.z / 4096, 0, 0, 0, Q/*new_Q*/, dt);

	//更新四元数(9轴IMMU-Inertial and Magnetic Measurement Units)
	//MahonyAHRSupdate(getValue[3] * PI/180, getValue[4] * PI/180, getValue[5] * PI/180,		 // PI/180：角度转弧度
	//		         getValue[0], getValue[1], getValue[2], getValue[6], getValue[7], getValue[8],Q);

	angles[0] = -fast_atan2(2 * Q[1] * Q[2] + 2 * Q[0] * Q[3], -2 * Q[2] * Q[2] - 2 * Q[3] * Q[3] + 1) * 180 / PI;  // yaw 
//	angles[1] = -asin(-2 * Q[1] * Q[3] + 2 * Q[0] * Q[2]) * 180 / PI;                                          // pitch	/*由此式可知以MPU6500的Y轴的负半轴为X轴*/
//	angles[2] = fast_atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1) * 180 / PI;   // roll	/*由此式可知以MPU6500的X轴为Y轴*/

	/*-----以下ROLL、PITCH所绕X、Y轴跟MPU6500的X、Y轴一致---*/
	angles[2] = asin(-2 * Q[1] * Q[3] + 2 * Q[0] * Q[2]) * 180 / PI;                                        // roll(Y轴)	 
	angles[1] = atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1) * 180 / PI;   // pitch(X轴)
	/*------------------------------------------------------*/

	// if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度	

	//angles[3] = -fast_atan2(2 * new_Q[1] * new_Q[2] + 2 * new_Q[0] * new_Q[3], -2 * new_Q[2] * new_Q[2] - 2 * new_Q[3] * new_Q[3] + 1) * 180 / PI;  // yaw 
	//angles[5] = asin(-2 * new_Q[1] * new_Q[3] + 2 * new_Q[0] * new_Q[2]) * 180 / PI;                                        // roll(Y轴)	 
	//angles[4] = atan2(2 * new_Q[2] * new_Q[3] + 2 * new_Q[0] * new_Q[1], -2 * new_Q[1] * new_Q[1] - 2 * new_Q[2] * new_Q[2] + 1) * 180 / PI;   // pitch(X轴)

	Yaw   = angles[0];
	Pitch = angles[1];
	Roll  = angles[2];

	//Yaw_NEW_XILUNA   = angles[3];
	//Pitch_NEW_XILUNA = angles[4];
	//Roll_NEW_XILUNA  = angles[5];
}
//====================================================================================================
// Functions:更新四元数

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update  (IMMU)

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement (对加速度进行归一化，以便接下来做叉乘时可直接得到sinθ)
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement  (对磁力计进行归一化，以便接下来做叉乘时可直接得到sinθ)
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field(同样是利用姿态矩阵将机体坐标系上的磁力分量变换到地理坐标系上)
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));	   //地理X轴磁力量
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));	   //地理Y轴磁力量
		bx = sqrt(hx * hx + hy * hy);														   //地理水平面磁力计的模长(在接下来的计算中将磁力在XY水平面的模长移到X轴上作为理论磁力分量)
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));	   //地理Z轴磁力量

		// Estimated direction of gravity and magnetic field 
		// 得到从地理坐标系({ 0, 0, 1 }^T)转到机体坐标系下的【理论重力向量】的一半(在twoKp、twoKi有乘以2倍)
		// 使用矩阵:C(b,R) = (C(R,b))^T：姿态矩阵的转置
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		// 得到从地理坐标系({ mx,0,mz }^T)转到机体坐标系下的【理论磁力向量】的一半(在twoKp、twoKi有乘以2倍) PS:我觉得是以正北为偏航角的0°基准 
		// 使用矩阵:C(b,R) = (C(R,b))^T：姿态矩阵的转置
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors  单位重力加速度、磁力向量各自叉乘得到角度误差的一半(在角度θ小于45°时,sinθ = θ)
		/*halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);*/
#if	  MPU6500_Coordinate
		halfex = (-ay * halfvz + az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (-az * halfvx + ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (-ax * halfvy + ay * halfvx) + (mx * halfwy - my * halfwx);
#else
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
#endif

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
		/* 至此我们就得到了由加速度计和磁力计修正过后的陀螺仪数据 */
	}

	// Integrate rate of change of quaternion 使用一阶龙格库塔（Runge-Kutta）更新四元数
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion	(由秦永元的《惯性导航》可知，四元数在求解过程中会渐渐失去规范性，因此需要隔段时间进行归一化)
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	Q[0] = q0;
	Q[1] = q1;
	Q[2] = q2;
	Q[3] = q3;
}

#define Q_xx  0.000001f
#define Q_yy  0.000001f
#define Q_zz  0.000001f
#define Q_xyz 0.00000001f
#define R_xx  0.00001f
#define R_yy  0.00001f
#define R_zz  0.000001f   /*0.001f*/
arm_matrix_instance_f32 PHT_type;
arm_matrix_instance_f32 HPHT_type;
arm_matrix_instance_f32 HPHT_R_type;
arm_matrix_instance_f32 K_type;
arm_matrix_instance_f32 P_type;
arm_matrix_instance_f32 P_type_temp;
float PHT_matrix[3][3]    = { 0,0,0,  0,0,0,  0,0,0 };
float HPHT_matrix[3][3]   = { 0,0,0,  0,0,0,  0,0,0 };
float HPHT_R_matrix[3][3] = { 0,0,0,  0,0,0,  0,0,0 };
float K_matrix[3][3]      = { 0,0,0,  0,0,0,  0,0,0 };
float P_matrix[3][3]      = { Q_xx,0,0,  0,Q_yy,0,  0,0,Q_zz };
//float P_matrix[3][3] = { Q_xx,Q_xyz,Q_xyz,  Q_xyz,Q_yy,Q_xyz,  Q_xyz,Q_xyz,Q_zz };
float P_matrix_temp[3][3] = { 0,0,0,     0,0,0,     0,0,0 };
float ewx = 0, ewy = 0, ewz = 0;
void Kalman_IMMU_Init(void)
{
	arm_mat_init_f32(&PHT_type,    3, 3, ((float *)PHT_matrix));
	arm_mat_init_f32(&HPHT_type,   3, 3, ((float *)HPHT_matrix));
	arm_mat_init_f32(&HPHT_R_type, 3, 3, ((float *)HPHT_R_matrix));
	arm_mat_init_f32(&K_type,      3, 3, ((float *)K_matrix));
	arm_mat_init_f32(&P_type,      3, 3, ((float *)P_matrix));
	arm_mat_init_f32(&P_type_temp, 3, 3, ((float *)P_matrix_temp));
}
void Kalman_IMMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q, float dt)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//float hx, hy, bx, bz;
	//float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	//float halfex, halfey, halfez;
	float qa, qb, qc;


	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					
	
	float Z_HX_xx = 0, Z_HX_yy = 0, Z_HX_zz = 0;
	//static float ewx = 0, ewy = 0, ewz = 0;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	

	// Normalise accelerometer measurement (对加速度进行归一化，以便接下来做叉乘时可直接得到sinθ)
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;

	// Normalise magnetometer measurement  (对磁力计进行归一化，以便接下来做叉乘时可直接得到sinθ)
	recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= recipNorm;
	my *= recipNorm;
	mz *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Estimated direction of gravity and magnetic field 
	// 得到从地理坐标系({ 0, 0, 1 }^T)转到机体坐标系下的【理论重力向量】的一半(在twoKp、twoKi有乘以2倍)
	// 使用矩阵:C(b,R) = (C(R,b))^T：姿态矩阵的转置
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = 2 * (q0q0 + q3q3) - 1.0f;

	ex = (ay * vz - az * vy) /*+ (my * halfwz - mz * halfwy)*/;
	ey = (az * vx - ax * vz) /*+ (mz * halfwx - mx * halfwz)*/;
	ez = (ax * vy - ay * vx) /*+ (mx * halfwy - my * halfwx)*/;

	/* 第一公式: X(k|k-1)=AX(k-1|k-1)+BU(k)
	| ewx |	  | 1   0   0 | | ewx | 
	| ewy | = | 0	1	0 | | ewy |
	| ewz |	  | 0 	0   1 | | ewz |	
	*/

	/* 第二公式: P(k|k-1)=AP(k-1|k-1)A'+Q
	
	*/
	P_matrix[0][0] += Q_xx;/*P_matrix[0][1] += Q_xyz;P_matrix[0][2] += Q_xyz;*/
	P_matrix[1][1] += Q_yy;/*P_matrix[1][0] += Q_xyz;P_matrix[1][2] += Q_xyz;*/
	P_matrix[2][2] += Q_zz;/*P_matrix[2][0] += Q_xyz;P_matrix[2][1] += Q_xyz;*/

	/* 第三公式: Kg(k)= P(k|k-1)H'/(HP(k|k-1)H'+R)
	
	*/
	memcpy(((float *)PHT_matrix),  ((float *)P_matrix), sizeof(P_matrix));
	memcpy(((float *)HPHT_matrix), ((float *)P_matrix), sizeof(P_matrix));
	HPHT_matrix[0][0] += R_xx;
	HPHT_matrix[1][1] += R_yy;
	HPHT_matrix[2][2] += R_zz;

	arm_mat_inverse_f32(&HPHT_type, &HPHT_R_type);
	arm_mat_mult_f32(&PHT_type, &HPHT_R_type, &K_type);

	/* 第四公式: X(k|k)= X(k|k-1)+Kg(k)(Z(k)-HX(k|k-1))
	
	*/
	Z_HX_xx = ex - ewx;
	Z_HX_yy = ey - ewy;
	Z_HX_zz = ez - ewz;

	ewx += K_matrix[0][0] * Z_HX_xx + K_matrix[0][1] * Z_HX_yy + K_matrix[0][2] * Z_HX_zz;
	ewy += K_matrix[1][0] * Z_HX_xx + K_matrix[1][1] * Z_HX_yy + K_matrix[1][2] * Z_HX_zz;
	ewz += K_matrix[2][0] * Z_HX_xx + K_matrix[2][1] * Z_HX_yy + K_matrix[2][2] * Z_HX_zz;

	/* 第五公式: P(k|k)=(I-Kg(k)H)P(k|k-1)
	
	*/
	K_matrix[0][0] = 1 - K_matrix[0][0];
	K_matrix[1][1] = 1 - K_matrix[1][1];
	K_matrix[2][2] = 1 - K_matrix[2][2];

	arm_mat_mult_f32(&K_type, &P_type, &P_type_temp);

	memcpy(((float *)P_matrix), ((float *)P_matrix_temp), sizeof(P_matrix_temp));


	// Apply proportional feedback
	gx += ewx;
	gy += ewy;
	gz += ewz;

	// Integrate rate of change of quaternion 使用一阶龙格库塔（Runge-Kutta）更新四元数
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion	(由秦永元的《惯性导航》可知，四元数在求解过程中会渐渐失去规范性，因此需要隔段时间进行归一化)
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	Q[0] = q0;
	Q[1] = q1;
	Q[2] = q2;
	Q[3] = q3;
}
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
#define betaDef		0.1f		// 2 * proportional gain
#define UT_ALPHA	1000
volatile float beta = betaDef;								// 2 * proportional gain (Kp)
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
	static float wq0 = 1.0f, wq1 = 0.0f, wq2 = 0.0f, wq3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
	float aq0 = 0.0f, aq1 = 0.0f, aq2 = 0.0f, aq3 = 0.0f;
	static float dt = 1.0f / sampleFreq;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, Q);
		return;
	}

	// Rate of change of quaternion from gyroscope	/* 使用一阶龙格库塔（Runge-Kutta）更新四元数(还没乘上dt) */
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	wq0 += qDot1 * (dt);
	wq1 += qDot2 * (dt);
	wq2 += qDot3 * (dt);
	wq3 += qDot4 * (dt);

	// Normalise quaternion
	recipNorm = invSqrt(wq0 * wq0 + wq1 * wq1 + wq2 * wq2 + wq3 * wq3);
	wq0 *= recipNorm;
	wq1 *= recipNorm;
	wq2 *= recipNorm;
	wq3 *= recipNorm;

	aq0 = UT_ALPHA * ABS(wq0);
	aq1 = UT_ALPHA * ABS(wq1);
	aq2 = UT_ALPHA * ABS(wq2);
	aq3 = UT_ALPHA * ABS(wq3);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	//q0 += qDot1 * (dt);
	//q1 += qDot2 * (dt);
	//q2 += qDot3 * (dt);
	//q3 += qDot4 * (dt);
	q0 += qDot1 * (dt) * aq0 / (aq0 + beta);
	q1 += qDot2 * (dt) * aq1 / (aq1 + beta);
	q2 += qDot3 * (dt) * aq2 / (aq2 + beta);
	q3 += qDot4 * (dt) * aq3 / (aq3 + beta);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	Q[0] = q0;
	Q[1] = q1;
	Q[2] = q2;
	Q[3] = q3;
}
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float *Q)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
	static float wq0 = 1.0f, wq1 = 0.0f, wq2 = 0.0f, wq3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
	float aq0 = 0.0f, aq1 = 0.0f, aq2 = 0.0f, aq3 = 0.0f;
	static float dt = 1.0f / sampleFreq;

	// Rate of change of quaternion from gyroscope(一阶龙格库塔(Runge-Kutta))
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	wq0 += qDot1 * (dt);
	wq1 += qDot2 * (dt);
	wq2 += qDot3 * (dt);
	wq3 += qDot4 * (dt);

	// Normalise quaternion
	recipNorm = invSqrt(wq0 * wq0 + wq1 * wq1 + wq2 * wq2 + wq3 * wq3);
	wq0 *= recipNorm;
	wq1 *= recipNorm;
	wq2 *= recipNorm;
	wq3 *= recipNorm;

	aq0 = UT_ALPHA * ABS(wq0);
	aq1 = UT_ALPHA * ABS(wq1);
	aq2 = UT_ALPHA * ABS(wq2);
	aq3 = UT_ALPHA * ABS(wq3);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement (对加速度进行归一化，以便接下来做叉乘时可直接得到sinθ)
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step  (梯度下降)
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude	 (获得梯度的方向)
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion (一阶龙格库塔(Runge-Kutta))
	//q0 += qDot1 * (dt);
	//q1 += qDot2 * (dt);
	//q2 += qDot3 * (dt);
	//q3 += qDot4 * (dt);
	q0 += qDot1 * (dt)* aq0 / (aq0 + beta);
	q1 += qDot2 * (dt)* aq1 / (aq1 + beta);
	q2 += qDot3 * (dt)* aq2 / (aq2 + beta);
	q3 += qDot4 * (dt)* aq3 / (aq3 + beta);

	// Normalise quaternion	(由秦永元的《惯性导航》可知，四元数在求解过程中会渐渐失去规范性，因此需要隔段时间进行归一化)
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	Q[0] = q0;
	Q[1] = q1;
	Q[2] = q2;
	Q[3] = q3;
}
#endif

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

