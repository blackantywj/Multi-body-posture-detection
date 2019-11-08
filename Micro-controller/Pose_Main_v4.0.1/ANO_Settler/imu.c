#include "imu.h"


#define Kp 0.3f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

float Roll, Pitch, Yaw;    				//姿态角

#if POSE_IMU

xyz_f_t reference_v;
ref_t 	ref;

//xyz_f_t Gravity_Vec;  				//解算的重力向量
	
float Roll,Pitch,Yaw;    				//姿态角
/*----------------------------------------------------
Roll：  -180°~ 180°
Pitch:  -90° ~ 90°
Yaw:    -180°~ 180°	(0°指向北方)
----------------------------------------------------*/
float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;

float mag_norm ,mag_norm_xyz ;

xyz_f_t mag_sim_3d,acc_3d_hg,acc_ng,acc_ng_offset;

u8 acc_ng_cali;
u8 imu_ready = 0;
//extern u8 fly_ready;
//得到Roll,Pitch,Yaw
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;
	static xyz_f_t mag_tmp;
	static float yaw_mag;
	
	mag_norm_tmp = 20 *(6.28f *half_T);	   /*0.2520792*/
	
	/*磁力计的模*/
	mag_norm_xyz = my_sqrt(mpu9250.Mag_Val.x * mpu9250.Mag_Val.x + mpu9250.Mag_Val.y * mpu9250.Mag_Val.y + mpu9250.Mag_Val.z * mpu9250.Mag_Val.z);
	
	if( mag_norm_xyz != 0) /*归一化后积分？*/
	{
		mag_tmp.x += mag_norm_tmp *( (float)mpu9250.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)mpu9250.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);
		mag_tmp.z += mag_norm_tmp *( (float)mpu9250.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);
	}

	/*
	void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out)
	
	罗盘数据是机体坐标下的，且磁场方向不是平行于地面，如果飞机倾斜，投影计算的角度会存在误差。
	此函数可在一定范围内做近似转换，让结果逼近实际角度，减小飞机倾斜的影响。
	注意：该函数内的计算并不是正确也不是准确的，正确的计算相对复杂，这里不给出，在未来的版本中会再更新。
	*/
	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d);  /*mag_sim_3d->算出较为正确的磁力方向*/
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
	
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		yaw_mag = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f;   /*通过磁力计求出飞控的偏航角的基准(指向北)*/
		
	}
	//=============================================================================
	//四元数性质:ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3] = 1;
	// 计算等效(理论)重力向量(模是1->单位向量(只关心理论重力向量的方向，不关心大小))	 矩阵:C(b,R) = (C(R,b))^T
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	
	//这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	//根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	//所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。       
	//=============================================================================

	if(acc_ng_cali)		 /*解锁后校准*/
	{
		if(acc_ng_cali==2)
		{
			acc_ng_offset.x = 0;
			acc_ng_offset.y = 0;
			acc_ng_offset.z = 0;
		}
			
		acc_ng_offset.x += 10 *TO_M_S2 *(ax - 4096*reference_v.x) *0.0125f ;
		acc_ng_offset.y += 10 *TO_M_S2 *(ay - 4096*reference_v.y) *0.0125f ;
		acc_ng_offset.z += 10 *TO_M_S2 *(az - 4096*reference_v.z) *0.0125f ;	
		
		acc_ng_cali ++;
		if(acc_ng_cali>=82) //start on 2	  (160ms)
		{
			acc_ng_cali = 0;
		}
	}
	
	acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x; /*矫正加速度，并换算单位成mm/s^2*/
	acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y; /*矫正加速度，并换算单位成mm/s^2*/
	acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z; /*矫正加速度，并换算单位成mm/s^2*/
	
	acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;	/*acc_3d_hg.z 约等于 acc_ng.z ,用来超声波融合*/
	

	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   

	/*通过加速度传感器获得的角度信息对陀螺仪的角度信息的积累误差进行校正*/
	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )
		{
			/* 单位重力加速度向量叉乘得到角度误差(在角度θ小于45°时,sinθ = θ) */
			//ax,ay,az:	实际单位重力加速度向量(加速度计)
			//reference_v.x,reference_v.y,reference_v.z:理论单位重力加速度向量(四元数换算成《方向余弦矩阵》中的第三列的三个元素)
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
	      //ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;  (Z轴(偏航角)会飘)
			
			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
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
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;	  /*T = 2 *half_T;*/
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* 角度积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
	if( reference_v.z > 0.0f )
	{
		if(imu_ready/*fly_ready*/  )
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
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;     //IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		//IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;
	
	/* 用叉积误差来做PI修正陀螺零偏 */

	/*四元数的求解：*/
	// integrate quaternion rate and normalise	/*使用一阶龙格库塔（Runge-Kutta）更新四元数*/
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;  

	/* 四元数规一化 (只要旋转信息，不平移)normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	
	/*求Roll/Pitch/Yaw,Z-Y-X欧拉角顺序*/
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;	/*没有负号?*/

	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
	//*yaw = yaw_mag;

}

#endif

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

