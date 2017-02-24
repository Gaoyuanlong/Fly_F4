#include "Position.h"

void Position_Init(void);
void Position_Updata(u16 Time);

Pos_Data_ Pos_Data = 
{
	0,0,0,
	0,0,0
};

struct Position_ Position = 
{
	&Pos_Data,
	False,
	Position_Init,
	Position_Updata
};


Filter_Balance FUSE_ALT_ACC_SPE(1,0,0);
Filter_Balance FUSE_ALT_ULTRA_SPE(10,0,0);
Filter_Balance FUSE_ALT_ULTRA_POS(1,0,0);
Filter_Balance FUSE_ALT_MS5611_POS(1,0,0);


float MS5611_Fused_Pos;
float Ultra_Fused_Pos;
float ACC_Fused_Spe;


void Position_Init(void)
{
	MS5611_SPI.Init();
	GPS.Init();
}

void MS5611_Fuse_Updata(u16 Time,float ACC_Earth)
{
	
	float Alt = 0;
	
	Alt = MS5611_SPI.Data->Altitude;
		
	Alt = FUSE_ALT_MS5611_POS.BalanceFilter(ACC_Fused_Spe,Alt,(float)Time / 1000);	
	
	MS5611_Fused_Pos = Alt;
	
}

BOOL Ultra_Fuse_Updata(u16 Time,float ACC_Earth)
{
	
	float Speed = 0;;	
	float Alt = 0;

	
	Alt = Ultra.Altitude;
	Speed = Ultra.Speed;	
	
	Speed = FUSE_ALT_ULTRA_SPE.BalanceFilter(ACC_Earth,Speed,(float)Time / 1000);
	// 超声波估计的速度与加速度计估计的速度相差过大时，认为超声波测量出错
	if(fabs(ACC_Fused_Spe - Speed) > Math.Constrain(fabs(ACC_Fused_Spe) + 0.2f,1,0)) return False; 
		
	Ultra_Fused_Pos = FUSE_ALT_ULTRA_POS.BalanceFilter(Speed,Alt,(float)Time / 1000);// * Z_Earth.z;	

	return True;
}
Filter_2nd ACCacc_Filter(0.1883633f,0,0,-1.023694f,0.2120577f);


void ACC_Fuse_Updata(u16 Time,float ACC_Earth)
{
	static float ACC_Fused_Spe_Bias = 0;
	
	float Tmp = FUSE_ALT_ACC_SPE.BalanceFilter(ACC_Earth,0,(float)Time / 1000);	
	
	float TimeNow = SystemTime.Now_MS();
	if(TimeNow > 10000)
	{
		if(FUSE_ALT_ACC_SPE.Kp == 1)
		{
			//加速收敛
			FUSE_ALT_ACC_SPE.Output *= 10;
			FUSE_ALT_ACC_SPE.Kp /= 10;
			ACC_Fused_Spe_Bias = FUSE_ALT_ACC_SPE.Output;
		}
		Position.IsReady = True;
	}
	else
	{
		Position.IsReady = False;
	}
	ACC_Fused_Spe = Tmp - ACC_Fused_Spe_Bias;

}

#define Kp 1.0f
#define Ki 0.0f
Quaternion Pos_Q;
void Updata_Pos_Quaternion(Vector RTK_GPS_Speed,Vector ACC,double DltaT)
{
	float Norm;
	double HalfT = DltaT / 2.0f;
	double vx,vy,vz;
	double ex,ey,ez;
	double gx,gy,gz;
	double ax,ay,az;
	static double exInt = 0, eyInt = 0, ezInt = 0;//定义姿态解算误差的积分
	static Vector Pos_Spe_Pre;
	Vector ACC_Earth;

	/***************************************************
	参数gx，gy，gz分别对应三个轴的位移速度，单位是米/秒（地理坐标系）
　参数ax，ay，az分别对应三个轴的加速度原始数据（地理坐标系）
	***************************************************/
	ACC_Earth = Math.Body_To_Earth(ACC,Attitude.Angle->y,Attitude.Angle->x,RTK_GPS.TrackAngle);
	ax = ACC_Earth.x;
	ay = ACC_Earth.y;
	az = ACC_Earth.z;

	gx = RTK_GPS_Speed.x;
	gy = RTK_GPS_Speed.y;
	gz = RTK_GPS_Speed.z;	
	
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
	vx = 2.0f * (Pos_Q.q2 * Pos_Q.q4 - Pos_Q.q1 * Pos_Q.q3);
	vy = 2.0f * (Pos_Q.q1 * Pos_Q.q2 + Pos_Q.q3 * Pos_Q.q4);
	vz = 1.0f - 2.0f * ( Pos_Q.q2 * Pos_Q.q2 + Pos_Q.q3 * Pos_Q.q3);//Q.w * Q.w + Q.z * Q.z;
	
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
	Pos_Q.q1 += (-Pos_Q.q2 * gx - Pos_Q.q3 * gy - Pos_Q.q4 * gz) * HalfT;
	Pos_Q.q2 += ( Pos_Q.q1 * gx + Pos_Q.q3 * gz - Pos_Q.q4 * gy) * HalfT;
	Pos_Q.q3 += ( Pos_Q.q1 * gy - Pos_Q.q2 * gz + Pos_Q.q4 * gx) * HalfT;
  Pos_Q.q4 += ( Pos_Q.q1 * gz + Pos_Q.q2 * gy - Pos_Q.q3 * gx) * HalfT;

	//四元数单位化
	arm_sqrt_f32(Pos_Q.q1 * Pos_Q.q1 + Pos_Q.q2 * Pos_Q.q2 + Pos_Q.q3 * Pos_Q.q3 + Pos_Q.q4 * Pos_Q.q4,&Norm);
	
	if(Norm == 0) return;
	
	Pos_Q.q1 = Pos_Q.q1 / Norm;
	Pos_Q.q2 = Pos_Q.q2 / Norm;
	Pos_Q.q3 = Pos_Q.q3 / Norm;
	Pos_Q.q4 = Pos_Q.q4 / Norm;		
	
	Pos_Data.POS_X = Degrees(atan2f(2.0f*(Pos_Q.q1*Pos_Q.q2 + Pos_Q.q3*Pos_Q.q4),1 - 2.0f*(Pos_Q.q2*Pos_Q.q2 + Pos_Q.q3*Pos_Q.q3)));
	Pos_Data.POS_Y = Degrees(Safe_Asin(2.0f*(Pos_Q.q1*Pos_Q.q3 - Pos_Q.q2*Pos_Q.q4)));
	Pos_Data.POS_Z = Degrees(atan2f(2.0f*(Pos_Q.q2*Pos_Q.q3 - Pos_Q.q1*Pos_Q.q4),2.0f*(Pos_Q.q1*Pos_Q.q1 + Pos_Q.q2*Pos_Q.q2) - 1));
	
	Pos_Data.SPE_X = (Pos_Data.POS_X - Pos_Spe_Pre.x)/ HalfT;
	Pos_Data.SPE_Y = (Pos_Data.POS_Y - Pos_Spe_Pre.y)/ HalfT;
	Pos_Data.SPE_Z = (Pos_Data.POS_Z - Pos_Spe_Pre.z)/ HalfT;
	
	Pos_Spe_Pre.x = Pos_Data.POS_X;
	Pos_Spe_Pre.y = Pos_Data.POS_Y;
	Pos_Spe_Pre.z = Pos_Data.POS_Z;
}

void Position_Updata(u16 Time)
{
	double DltaT;
	uint64_t Time_Now = 0;
	static uint64_t Time_Pre = 0;
	
	Time_Now = SystemTime.Now_US();
	DltaT = (Time_Now - Time_Pre) * (double)1e-6;
	Time_Pre = Time_Now;
	
	Updata_Pos_Quaternion(RTK_GPS_Speed,MPU6050.Data->ACC_ADC,DltaT);	
}



