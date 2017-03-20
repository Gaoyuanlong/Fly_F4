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
	USART1_Cf.Init(115200);
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

#define posKp 0.01f
#define posKi 0.0f
#define spKp 0.01f
#define spKi 0.0f
#define Earth_G 4096
void Updata_Pos_Quaternion(RTK_XYZ_HP_ RTK_XYZ_HP,Vector ACC,u16 DltaT)
{
	float DltaT_S = DltaT/1000.0f;
	float RTKx,RTKy,RTKz;
	float POSex,POSey,POSez;
	float SPex,SPey,SPez;
	float Px,Py,Pz;
	float ax,ay,az;
	static float exInt = 0, eyInt = 0, ezInt = 0;//定义姿态解算误差的积分
	Vector ACC_Earth;
	Vector SPE_Temp(0,0,0);
	static Vector POS_Temp(0,0,0);

	/***************************************************
　状态就绪
	***************************************************/	
	float TimeNow = SystemTime.Now_MS();
	if(TimeNow > 10000)
	{
		Position.IsReady = True;
	}
	else
	{
		Position.IsReady = False;
	}
	/***************************************************
　参数ax，ay，az分别对应三个轴的加速度原始数据（地理坐标系）
	***************************************************/
	ACC_Earth = Math.Body_To_Earth(ACC,Attitude.Angle->y,Attitude.Angle->x,Attitude.Angle->z);
	ax = ACC_Earth.x;
	ay = ACC_Earth.y;
	az = ACC_Earth.z - Earth_G;//减掉重力分量
	
	ax = ax/Earth_G*9.8f;
	ay = ay/Earth_G*9.8f;
	az = az/Earth_G*9.8f;

//	extern u32 SendData[8];
//		SendData[0] = ACC_Earth.x;
//		SendData[1] = ACC_Earth.y;
//		SendData[2] = ACC_Earth.z;
	/**************************************************
	位置取值
	***************************************************/
	Px = Pos_Data.POS_X;
	Py = Pos_Data.POS_Y;
	Pz = Pos_Data.POS_Z;
	
	RTKx = RTK_XYZ_HP.Lon_M;
	RTKy = RTK_XYZ_HP.Lat_M;
	RTKz = RTK_XYZ_HP.Alt_M;	
	/**************************************************
	推算速度，速度误差
	***************************************************/
	SPE_Temp.x = (RTKx - POS_Temp.x)/(DltaT_S);
	SPE_Temp.y = (RTKy - POS_Temp.y)/(DltaT_S);
	SPE_Temp.z = (RTKz - POS_Temp.z)/(DltaT_S);
	
	POS_Temp.x = RTKx;//存历史值
	POS_Temp.y = RTKy;
	POS_Temp.z = RTKz;
	
	SPex = SPE_Temp.x - Pos_Data.SPE_X;
	SPey = SPE_Temp.y - Pos_Data.SPE_Y;
	SPez = SPE_Temp.z - Pos_Data.SPE_Z;
	/**************************************************
	位置误差
	***************************************************/
	POSex = (RTKx - Px );
	POSey = (RTKy - Py );
	POSez = (RTKz - Pz );
	
	/***************************************************
	修正速度，修正位置
	***************************************************/
	Pos_Data.SPE_X += ax * DltaT_S + spKp * SPex;//速度反馈 修正：加速度，得到一次修正速度
	Pos_Data.SPE_Y += ay * DltaT_S + spKp * SPey;
	Pos_Data.SPE_Z += az * DltaT_S + spKp * SPez;

//	Pos_Data.SPE_X = Pos_Data.SPE_X + posKp * POSex;//位置反馈 修正：速度，得到二次修正速度
//	Pos_Data.SPE_Y = Pos_Data.SPE_Y + posKp * POSey;
//	Pos_Data.SPE_Z = Pos_Data.SPE_Z + posKp * POSez;
	
	Pos_Data.POS_X += Pos_Data.SPE_X * DltaT_S + posKp * POSex;
	Pos_Data.POS_Y += Pos_Data.SPE_Y * DltaT_S + posKp * POSey;
	Pos_Data.POS_Z += Pos_Data.SPE_Z * DltaT_S + posKp * POSez;
}

void Position_Updata(u16 Time)
{
	Updata_Pos_Quaternion(RTK_XYZ_HP,MPU6050.Data->ACC_ADC,Time);	
}



