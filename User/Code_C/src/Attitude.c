#include "Attitude.h"
#define Kp 1.0f
#define Ki 0.0f

Filter_2nd ATT_Filter(0.1883633f,0,0,-1.023694f,0.2120577f);
Quaternion Q;//,Qz;
u8 IsCalulate = True;
struct Vector Angle;
struct Vector Rate;

/*
	坐标系关系说明：
	机体坐标系：以MPU6050芯片的坐标系一致  -> PWM捕获接口方向是机头方向
	            顺着坐标轴方向看去，顺时针方向为正方向
	            Pitch ： 绕坐标系Y轴运动		Vector Angle.y
	            Roll  ： 绕坐标系X轴运动		Vector Angle.x
							Yaw   ： 绕坐标系Z轴运动   Vector Angle.z
	参考坐标系：北东天坐标系  

*/


void Updata_Eular(void);

struct Attitude_ Attitude = 
{
	&Rate,
  &Angle,
	Updata_Eular,
};

void Updata_Quaternion(Vector GYR,Vector ACC,Vector MAG,double DltaT)
{
	float Norm;
	double HalfT = DltaT / 2.0f;
	double vx,vy,vz;
	double ex,ey,ez;
	double gx,gy,gz;
	double ax,ay,az;
	static double exInt = 0, eyInt = 0, ezInt = 0;//定义姿态解算误差的积分


	/***************************************************
	参数gx，gy，gz分别对应三个轴的角速度，单位是弧度/秒
　参数ax，ay，az分别对应三个轴的加速度原始数据
	***************************************************/
	
	ax = ACC.x;
	ay = ACC.y;
	az = ACC.z;

	gx = Radians(GYR.x * MPU6050_GYR_2000DPS);
	gy = Radians(GYR.y * MPU6050_GYR_2000DPS);
	gz = Radians(GYR.z * MPU6050_GYR_2000DPS);	
	
	Attitude.Rate->x = gx;
	Attitude.Rate->y = gy;
	Attitude.Rate->z = gz;
	
	//将加速度的原始数据，归一化，得到单位加速度
	arm_sqrt_f32(ax * ax + ay * ay + az * az,&Norm);
	if(Norm == 0) return;
	ax = ax / Norm;
	ay = ay / Norm;
	az = az / Norm;
	
	/**************************************************
	把四元数换算成“方向余弦矩阵”中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，
	转到机体坐标系，正好是这三个元素。所以这里的vx、vy、vz，
	其实就是当前的机体坐标参照系上，换算出来的重力单位向量。
	(用表示机体姿态的四元数进行换算)
	***************************************************/
	vx = 2.0f * (Q.q2 * Q.q4 - Q.q1 * Q.q3);
	vy = 2.0f * (Q.q1 * Q.q2 + Q.q3 * Q.q4);
	vz = 1.0f - 2.0f * ( Q.q2 * Q.q2 + Q.q3 * Q.q3);//Q.w * Q.w + Q.z * Q.z;
	
	/***************************************************
	向量间的误差，可以用向量积(也叫外积、叉乘)来表示，	ex、
	ey、ez就是两个重力向量的叉积。这个叉积向量仍旧是位于机体
	坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大
	小与陀螺积分误差成正比，正好拿来纠正陀螺。由于陀螺是对机
	体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的
	纠正。
	***************************************************/
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);
	
	/***************************************************
	用叉乘误差来做PI修正陀螺零偏，通过调节Kp，Ki两个参数，可
	以控制加速度计修正陀螺仪积分姿态的速度
	***************************************************/
	if(Ki > 0)
	{
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}
	else
	{
		gx = gx + Kp * ex;
		gy = gy + Kp * ey;
		gz = gz + Kp * ez;   
	}
	
	//四元数微分方程 
	Q.q1 += (-Q.q2 * gx - Q.q3 * gy - Q.q4 * gz) * HalfT;
	Q.q2 += ( Q.q1 * gx + Q.q3 * gz - Q.q4 * gy) * HalfT;
	Q.q3 += ( Q.q1 * gy - Q.q2 * gz + Q.q4 * gx) * HalfT;
	Q.q4 += ( Q.q1 * gz + Q.q2 * gy - Q.q3 * gx) * HalfT;

	//四元数单位化
	arm_sqrt_f32(Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3 + Q.q4 * Q.q4,&Norm);
	
	if(Norm == 0) return;
	
	Q.q1 = Q.q1 / Norm;
	Q.q2 = Q.q2 / Norm;
	Q.q3 = Q.q3 / Norm;
	Q.q4 = Q.q4 / Norm;		
	
	Angle.x = Degrees(atan2f(2.0f*(Q.q1*Q.q2 + Q.q3*Q.q4),1 - 2.0f*(Q.q2*Q.q2 + Q.q3*Q.q3)));
	Angle.y = Degrees(Safe_Asin(2.0f*(Q.q1*Q.q3 - Q.q2*Q.q4)));
	Angle.z = Degrees(atan2f(2.0f*(Q.q2*Q.q3 - Q.q1*Q.q4),2.0f*(Q.q1*Q.q1 + Q.q2*Q.q2) - 1));
	//坐标轴不重合，且存在反向
	//利用地磁计算偏航角速度
	//为避免Z轴的修正对俯仰角及翻滚角的影响，单独修正Z轴
	//地磁计数据在飞机倾斜时需要根据角度进行补偿
	//绕y轴运动->翻滚
	
//	if((MAG.x != 0) || (MAG.y != 0)) 
//	{
//		//板载地磁
//		float MagX,MagY,AngleZ;
//		float COS_P = arm_cos_f32(Radians(-Angle.y));
//		float COS_R = arm_cos_f32(Radians(Angle.x));
//		float SIN_P = arm_sin_f32(Radians(-Angle.y));
//		float SIN_R = arm_sin_f32(Radians(Angle.x));
//		
//		MagX = HMC5883.Data->MAG_ADC.x * COS_P + HMC5883.Data->MAG_ADC.y * SIN_R * SIN_P - HMC5883.Data->MAG_ADC.z * COS_R * SIN_P;
//		MagY = HMC5883.Data->MAG_ADC.y * COS_R - HMC5883.Data->MAG_ADC.z * SIN_R;
//		
//		AngleZ = Degrees(atan2f(MagX,MagY));
//		
//		if(abs(To_180_degrees(AngleZ - Angle.z)) < 1.0f)
//			gz -= 0.1f * Kp * Radians(To_180_degrees(AngleZ - Angle.z));
//		else
//			gz -= Kp * Radians(To_180_degrees(AngleZ - Angle.z));	
//	}
//	//四元数微分方程 
//	Qz.q1 += (-Qz.q2 * gx - Qz.q3 * gy - Qz.q4 * gz) * HalfT;
//	Qz.q2 += ( Qz.q1 * gx + Qz.q3 * gz - Qz.q4 * gy) * HalfT;
//	Qz.q3 += ( Qz.q1 * gy - Qz.q2 * gz + Qz.q4 * gx) * HalfT;
//	Qz.q4 += ( Qz.q1 * gz + Qz.q2 * gy - Qz.q3 * gx) * HalfT;

//	//四元数单位化
//	arm_sqrt_f32(Qz.q1 * Qz.q1 + Qz.q2 * Qz.q2 + Qz.q3 * Qz.q3 + Qz.q4 * Qz.q4,&Norm);
//	
//	if(Norm == 0) return;
//	
//	Qz.q1 = Qz.q1 / Norm;
//	Qz.q2 = Qz.q2 / Norm;
//	Qz.q3 = Qz.q3 / Norm;
//	Qz.q4 = Qz.q4 / Norm;

}

/***************************************************
* 更新欧拉角
* 每个2MS计算一次姿态角
***************************************************/

void Updata_Eular(void)
{
	double DltaT;
	uint64_t Time_Now = 0;
	static uint64_t Time_Pre = 0;
	
	Time_Now = SystemTime.Now_US();
	DltaT = (Time_Now - Time_Pre) * (double)1e-6;
	Time_Pre = Time_Now;
	
	Updata_Quaternion(MPU6050.Data->GYR_ADC,MPU6050.Data->ACC_ADC,HMC5883.Data->MAG_ADC,DltaT);	

}





